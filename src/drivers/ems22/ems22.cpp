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
 * @file ems22.cpp
 * @author Yoann Lapijover
 *
 * Driver for the ems22 encoder via an arduino nano, the connection is via I2C.
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

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/distance_sensor.h>

#include <systemlib/param/param.h>

#include <board_config.h>

 /**
 * Encoder offset
 *
 * @group encodeur driver
 */
PARAM_DEFINE_INT32(DRV_ENC_OFFSET, 0);

/* Configuration Constants */
#define EMS22_BUS           PX4_I2C_BUS_EXPANSION
#define EMS22_BASEADDR      0x30 /* 7-bit address */
#define EMS22_DEVICE_PATH   "/dev/encoder"

/* Nano Registers addresses */

#define EMS22_MEASURE_REG	0x00		/* Measure range register */
#define EMS22_WHO_AM_I_REG  0x01        /* Who am I test register */
#define EMS22_WHO_AM_I_REG_VAL 0xA1

/* Device limits */
#define EMS22_MIN_DISTANCE (0.0f)
#define EMS22_MAX_DISTANCE (0.5f)

#define EMS22_CONVERSION_INTERVAL 10000 /* 5ms */

/* Enable if you want a rolling average filter for the waves */
//#define ROLLING_AVERAGE_FILTER

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

class EMS22 : public device::I2C
{
public:
	EMS22(int bus = EMS22_BUS, int address = EMS22_BASEADDR);
	virtual ~EMS22();

	virtual int 		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	uint16_t			get_angle();

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:
	float				_min_distance;
	float				_max_distance;
	uint16_t 			_angle;
	uint16_t			_offset;
	work_s				_work;
	ringbuffer::RingBuffer		*_reports;
	bool				_sensor_ok;
	uint8_t				_valid;
	int					_measure_ticks;
	bool				_collect_phase;
	int				_class_instance;
	int				_orb_class_instance;

	orb_advert_t		_distance_sensor_topic;

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
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults EMS22_MIN_DISTANCE
	* and EMS22_MAX_DISTANCE
	*/
	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

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
extern "C" __EXPORT int ems22_main(int argc, char *argv[]);

EMS22::EMS22(int bus, int address) :
	I2C("EMS22", EMS22_DEVICE_PATH, bus, address, 100000),
	_min_distance(EMS22_MIN_DISTANCE),
	_max_distance(EMS22_MAX_DISTANCE),
	_reports(nullptr),
	_sensor_ok(false),
	_valid(0),
	_measure_ticks(0),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
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

EMS22::~EMS22()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
}

int
EMS22::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		DEVICE_LOG("fck");
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	if (_reports == nullptr) {
		goto out;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* get a publish handle on the range finder topic */
		struct distance_sensor_s ds_report;
		measure();
		_reports->get(&ds_report);

		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
					 &_orb_class_instance, ORB_PRIO_LOW);

		if (_distance_sensor_topic == nullptr) {
			DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
		}
	}

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;
out:
	return ret;
}

int
EMS22::probe()
{
	uint8_t who_am_i = 0;

	const uint8_t cmd = EMS22_WHO_AM_I_REG;

	// set the I2C bus address
	set_address(EMS22_BASEADDR);

	// can't use a single transfer as nano need a bit of time for internal processing
	if (transfer(&cmd, 1, nullptr, 0) == OK) {
		if (transfer(nullptr, 0, &who_am_i, 1) == OK && who_am_i == EMS22_WHO_AM_I_REG_VAL) {
			return measure();
		}
	}

	DEVICE_DEBUG("WHO_AM_I byte mismatch 0x%02x should be 0x%02x\n",
		     (unsigned)who_am_i,
		     EMS22_WHO_AM_I_REG_VAL);

	// not found on any address
	return -EIO;
}

void
EMS22::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
EMS22::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
EMS22::get_minimum_distance()
{
	return _min_distance;
}

float
EMS22::get_maximum_distance()
{
	return _max_distance;
}

uint16_t
EMS22::get_angle()
{
	return _angle;
}

int
EMS22::ioctl(struct file *filp, int cmd, unsigned long arg)
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
					_measure_ticks = USEC2TICK(EMS22_CONVERSION_INTERVAL);

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
					if (ticks < USEC2TICK(EMS22_CONVERSION_INTERVAL)) {
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

	case RANGEFINDERIOCSETMINIUMDISTANCE: {
			set_minimum_distance(*(float *)arg);
			return 0;
		}
		break;

	case RANGEFINDERIOCSETMAXIUMDISTANCE: {
			set_maximum_distance(*(float *)arg);
			return 0;
		}
		break;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
EMS22::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
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
		usleep(EMS22_CONVERSION_INTERVAL);

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
EMS22::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	const uint8_t cmd = EMS22_MEASURE_REG;
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
EMS22::collect()
{
	int ret = -EIO;

	/* read from the sensor */
	uint8_t val[2] = {0, 0};

	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		DEVICE_LOG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	param_get(param_find("DRV_ENC_OFFSET"), &_offset);

	_angle = -((val[1] << 8) | val[0]) + _offset;

    float angle_deg = _angle * 360.f / 1024.f;
    const float rod_length = 0.71f; /* meter */
	float distance_m = cosf(math::radians(angle_deg)) * rod_length;

	static float distance_buf[10];
	static int index = 0;
	distance_buf[index] = distance_m/10.f;
	index = (index + 1) % 10;

	float distance_average = 0.f;
	for (int i = 0; i < 10; ++i) {
		distance_average += distance_buf[i];
	}

	struct distance_sensor_s report;

	report.timestamp = hrt_absolute_time();
	/* there is no enum item for a combined LASER and ULTRASOUND which it should be */
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_MECHANICAL;
	report.orientation = 8;

#ifdef ROLLING_AVERAGE_FILTER
	report.current_distance = distance_average;
	report.raw_distance = distance_m;
#else
	report.raw_distance = distance_m;
	report.current_distance = report.raw_distance;
#endif

	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.covariance = 0.0f;
	/* TODO: set proper ID */
	report.id = 0;

	// This validation check can be used later
	_valid = (float)report.current_distance > report.min_distance
		 && (float)report.current_distance < report.max_distance ? 1 : 0;

	/* publish it, if we are the primary */
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}

	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
EMS22::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&EMS22::cycle_trampoline, this, 1);

	/* notify about state change */
	struct subsystem_info_s info = {};
	info.present = true;
	info.enabled = true;
	info.ok = true;
	info.subsystem_type = subsystem_info_s::SUBSYSTEM_TYPE_RANGEFINDER;

	static orb_advert_t pub = nullptr;

	if (pub != nullptr) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);

	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}

void
EMS22::stop()
{
	work_cancel(HPWORK, &_work);
}

void
EMS22::cycle_trampoline(void *arg)
{
	EMS22 *dev = (EMS22 *)arg;

	dev->cycle();
}

void
EMS22::cycle()
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
		if (_measure_ticks > USEC2TICK(EMS22_CONVERSION_INTERVAL)) {
			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&EMS22::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(EMS22_CONVERSION_INTERVAL));

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
		   (worker_t)&EMS22::cycle_trampoline,
		   this,
		   USEC2TICK(EMS22_CONVERSION_INTERVAL));
}

void
EMS22::print_info()
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
namespace ems22
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

EMS22	*g_dev;

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
	g_dev = new EMS22(EMS22_BUS);


	if (g_dev == nullptr) {
		errx(1, "did not create class");
		goto fail;
	}

	if (OK != g_dev->init()) {
		errx(1, "init failed");
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(EMS22_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		errx(1, "open failed");
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
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
	struct distance_sensor_s report;
	ssize_t sz;
	int ret;

	int fd = open(EMS22_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'ems22 start' if the driver is not running", EMS22_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %0.2f m", (double)report.current_distance);
	warnx("time:        %llu", report.timestamp);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 50x and report each value */
	for (unsigned i = 0; i < 50; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("measurement: %0.3f", (double)report.current_distance);
		warnx("minimum distance: %0.3f", (double)report.min_distance);
		warnx("maximum distance: %0.3f", (double)report.max_distance);
		warnx("time:        %llu", report.timestamp);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(EMS22_DEVICE_PATH, O_RDONLY);

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
	struct distance_sensor_s report;
	ssize_t sz;

	int fd = open(EMS22_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'ems22 start' if the driver is not running", EMS22_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %0.2f m", (double)report.current_distance);
	warnx("raw angle: %d ", g_dev->get_angle());
	warnx("time:        %llu", report.timestamp);
}

} // namespace

int
ems22_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		ems22::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		ems22::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		ems22::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		ems22::reset();
	}

	/*
	 * Calib the driver.
	 */
	if (!strcmp(argv[1], "calib")) {
		ems22::calib();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		ems22::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
