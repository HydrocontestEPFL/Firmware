/****************************************************************************
 *
 *   Copyright (c) 2016 Hydrocontest EPFL Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Hydrocontest Team EPFL nor the names of its
 *	  contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file pwrHC.cpp
 * @author Benjamin Bonnal
 *
 * Driver for the pwrHC encoder via an arduino nano, the connection is via I2C.
 */

#include <px4_config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <mathlib/mathlib.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_orb_dev.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/battery_status.h>

#include <systemlib/param/param.h>

#include <board_config.h>

 /**
 * Encoder offset
 *
 * @group encodeur driver
 */

/* Configuration Constants */
#define PWRHC_BUS           PX4_I2C_BUS_EXPANSION
#define PWRHC_BASEADDR      0x40 /* 7-bit address */
#define PWRHC_DEVICE_PATH   "/dev/pwrhc"

/* Nano Registers addresses */

#define PWRHC_MEASURE_REG	0x00		/* Measure range register */
#define PWRHC_WHO_AM_I_REG  0x01        /* Who am I test register */
#define PWRHC_WHO_AM_I_REG_VAL 0xA2


#define PWRHC_CONVERSION_INTERVAL 10000 /* 5ms */

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

class PWRHC : public device::I2C
{
public:
	PWRHC(int bus = PWRHC_BUS, int address = PWRHC_BASEADDR);
	virtual ~PWRHC();

	virtual int 		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);


	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:
	work_s				_work;
	ringbuffer::RingBuffer		*_reports;
	bool				_sensor_ok;
	uint8_t				_valid;
	int					_measure_ticks;
	bool				_collect_phase;
	int				_class_instance;
	int				_orb_class_instance;

	orb_advert_t		_battery_status_topic;
	orb_id_t		_batt_orb_id;	///< uORB battery topic ID

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return		True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();


	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int					measure();
	int					collect();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void		cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int PWRHC_main(int argc, char *argv[]);

PWRHC::PWRHC(int bus, int address) :
	I2C("PWRHC", PWRHC_DEVICE_PATH, bus, address, 100000),
	_reports(nullptr),
	_sensor_ok(false),
	_valid(0),
	_measure_ticks(0),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_battery_status_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "tr1_read")),
	_comms_errors(perf_alloc(PC_COUNT, "tr1_com_err")),
	_buffer_overflows(perf_alloc(PC_COUNT, "tr1_buf_of"))
{
	// up the retries since the device misses the first measure attempts
	I2C::_retries = 3;

	// enable debug() calls
	_debug_enabled = false;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

PWRHC::~PWRHC()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(PWRHC_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
}

int
PWRHC::init()
{

  int ret = ENOTTY;

	// attempt to initialise I2C bus
	ret = I2C::init();

	if (ret != OK) {
		errx(1, "failed to init I2C");
		return ret;

	} else {
		// allocate basic report buffers
		_reports = new ringbuffer::RingBuffer(2, sizeof(struct battery_status_s));

		if (_reports == nullptr) {
			ret = ENOTTY;

		} else {
			// start work queue
			start();
		}
	}

	// init orb id
	_batt_orb_id = ORB_ID(battery_status);

	return ret;
  /*
  int ret = ERROR;

	// do I2C init (and probe) first
	if (I2C::init() != OK) {
    errx(1, "i2c::init failed");
		goto out;
	}

	// allocate basic report buffers
	_reports = new ringbuffer::RingBuffer(2, sizeof(battery_status_s));

	if (_reports == nullptr) {
    errx(1, "ringbuffer creation failed");
		goto out;
	}

	_class_instance = register_class_devname(PWRHC_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		// get a publish handle on the range finder topic
		struct battery_status_s ds_report;
		measure();
		_reports->get(&ds_report);

		_battery_status_topic = orb_advertise_multi(ORB_ID(battery_status), &ds_report,
					 &_orb_class_instance, ORB_PRIO_LOW);

		if (_battery_status_topic == nullptr) {
			DEVICE_LOG("failed to create battery_status object. Did you start uOrb?");
		}
	}

	ret = OK;
	// sensor is ok, but we don't really know if it is within range
	_sensor_ok = true;
out:
	return ret;*/


}

int
PWRHC::probe()
{
	uint8_t who_am_i = 0;

	const uint8_t cmd = PWRHC_WHO_AM_I_REG;

	// set the I2C bus address
	set_address(PWRHC_BASEADDR);

	// can't use a single transfer as Arduino nano need a bit of time for internal processing
	if (transfer(&cmd, 1, nullptr, 0) == OK) {
		if (transfer(nullptr, 0, &who_am_i, 1) == OK && who_am_i == PWRHC_WHO_AM_I_REG_VAL) {
			return measure();
		}
	}

	DEVICE_DEBUG("WHO_AM_I byte mismatch 0x%02x should be 0x%02x\n",
		     (unsigned)who_am_i,
		     PWRHC_WHO_AM_I_REG_VAL);

	// not found on any address
	return -EIO;
}


int
PWRHC::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(PWRHC_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(PWRHC_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = irqsave();

			if (!_reports->resize(arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}

			irqrestore(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;


	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
PWRHC::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct battery_status_s);
	struct battery_status_s *rbuf = reinterpret_cast<struct battery_status_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(PWRHC_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
PWRHC::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	const uint8_t cmd = PWRHC_MEASURE_REG;
	ret = transfer(&cmd, sizeof(cmd), nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_LOG("i2c::transfer returned %d", ret);
		return ret;
	}

	ret = OK;

	return ret;
}

int
PWRHC::collect()
{
	int ret = -EIO;

	/* read from the sensor */
	uint8_t val[8] = {0,0,0,0,0,0,0,0};

	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], 8);

	if (ret < 0) {
		DEVICE_LOG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t volt_bundle = ((val[1] << 8) | val[0]);
  uint16_t a_bundle = ((val[3] << 8) | val[2]);
  uint16_t mah_bundle = ((val[5] << 8) | val[4]);



	struct battery_status_s report;

  report.connected = true;
  report.voltage_v = volt_bundle;
  report.voltage_filtered_v = volt_bundle;
  report.current_a = a_bundle;
  report.current_filtered_a = a_bundle;
  report.discharged_mah = mah_bundle;

	//report.timestamp = hrt_absolute_time();
	/* there is no enum item for a combined LASER and ULTRASOUND which it should be */
	/*report.type = _battery_status_s::MAV_battery_status_MECHANICAL;
	report.orientation = 8;
	report.current_distance = distance_average;
	report.raw_distance = distance_m;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.covariance = 0.0f;*/

	//report.id = 0;



  // publish it, if we are the primary
	/*if (_battery_status_topic != nullptr) {
		orb_publish(ORB_ID(battery_status), _battery_status_topic, &report);
	}

	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}*/

  // publish to orb
  if (_battery_status_topic != nullptr) {
    orb_publish(_batt_orb_id, _battery_status_topic, &report);

  } else {
    _battery_status_topic = orb_advertise(_batt_orb_id, &report);

    if (_battery_status_topic == nullptr) {
      errx(1, "ADVERT FAIL");
    }
  }

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
PWRHC::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&PWRHC::cycle_trampoline, this, 1);
}

void
PWRHC::stop()
{
	work_cancel(HPWORK, &_work);
}

void
PWRHC::cycle_trampoline(void *arg)
{
	PWRHC *dev = (PWRHC *)arg;

	dev->cycle();
}

void
PWRHC::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			DEVICE_LOG("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(PWRHC_CONVERSION_INTERVAL)) {
			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&PWRHC::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(PWRHC_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		DEVICE_LOG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&PWRHC::cycle_trampoline,
		   this,
		   USEC2TICK(PWRHC_CONVERSION_INTERVAL));
}

void
PWRHC::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace pwrHC
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

PWRHC	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();
void	calib();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new PWRHC(PWRHC_BUS);


	if (g_dev == nullptr) {
    errx(1, "fail1");
		goto fail;
	}

	if (OK != g_dev->init()) {
    errx(1, "fail2");
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(PWRHC_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
    errx(1, "fail3");
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
    errx(1, "fail4");
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{

	struct battery_status_s report;
	ssize_t sz;
	int ret;

	int fd = open(PWRHC_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'pwrHC start' if the driver is not running", PWRHC_DEVICE_PATH);
	}

	// do a simple demand read
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %0.2f m", (double)report.voltage_v);

	// start the sensor polling at 2Hz
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	// read the sensor 50x and report each value
	for (unsigned i = 0; i < 50; i++) {
		struct pollfd fds;

		// wait for data to be ready
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		// now go get it
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("voltage: %0.3f", (double)report.voltage_v);
    warnx("amp: %0.3f", (double)report.current_a);
    warnx("mah: %0.3f", (double)report.discharged_mah);

	}

	// reset the sensor polling to default rate
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	errx(0, "PASS");

  int sub = orb_subscribe(ORB_ID(battery_status));
	bool updated = false;
	struct battery_status_s status;
	uint64_t start_time = hrt_absolute_time();

	// loop for 5 seconds
	while ((hrt_absolute_time() - start_time) < 5000000) {

		// display new info that has arrived from the orb
		orb_check(sub, &updated);

		if (updated) {
			if (orb_copy(ORB_ID(battery_status), sub, &status) == OK) {
				warnx("V=%4.2f C=%4.2f", (double)status.voltage_v, (double)status.current_a);
			}
		}

		// sleep for 0.05 seconds
		usleep(50000);
	}

}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(PWRHC_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

void
calib()
{
	struct battery_status_s report;
	ssize_t sz;

	int fd = open(PWRHC_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'pwrHC start' if the driver is not running", PWRHC_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %0.2f m", (double)report.voltage_v);
}

} // namespace

int
PWRHC_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		pwrHC::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		pwrHC::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		pwrHC::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		pwrHC::reset();
	}

	/*
	 * Calib the driver.
	 */
	if (!strcmp(argv[1], "calib")) {
		pwrHC::calib();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		pwrHC::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
