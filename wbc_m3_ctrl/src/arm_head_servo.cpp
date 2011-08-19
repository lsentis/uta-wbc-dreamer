/*
 * Whole-Body Control for Human-Centered Robotics http://www.me.utexas.edu/~hcrl/
 *
 * Copyright (c) 2011 University of Texas at Austin. All rights reserved.
 *
 * Author: Josh Petersen
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <wbc_m3_ctrl/rt_util_dual.h>

// one of these just for logging timestamp
#include <rtai_sched.h>
#include <rtai_shm.h>
#include <rtai.h>
#include <rtai_sem.h>
#include <rtai_nam2num.h>
#include <rtai_registry.h>

#include <ros/ros.h>
#include <jspace/test/sai_util.hpp>
#include <opspace/Skill.hpp>
#include <opspace/Factory.hpp>
#include <uta_opspace/ControllerNG.hpp>
#include <uta_opspace/HelloGoodbyeSkill.hpp>
#include <uta_opspace/TaskPostureSkill.hpp>
#include <uta_opspace/WriteSkill.hpp>
#include <uta_opspace/StaticAccuracyTest.hpp>
#include <uta_opspace/DynamicAccuracyTest.hpp>
#include <uta_opspace/TrajAccuracyTest.hpp>
#include <uta_opspace/CircleTest.hpp>
#include <uta_opspace/GestureSkill.hpp>
#include <uta_opspace/JointTest.hpp>
#include <wbc_core/opspace_param_callbacks.hpp>
#include <boost/scoped_ptr.hpp>
#include <err.h>
#include <signal.h>

using namespace wbc_m3_ctrl;
using namespace opspace;
using namespace wbc_core_opspace;
using namespace uta_opspace;
using namespace boost;
using namespace std;


static bool verbose(false);
static scoped_ptr<jspace::Model> arm_model;
static scoped_ptr<jspace::Model> head_model;
static shared_ptr<Factory> arm_factory;
static shared_ptr<Factory> head_factory;
static shared_ptr<opspace::ReflectionRegistry> arm_registry;
static shared_ptr<opspace::ReflectionRegistry> head_registry;
static long long servo_rate;
static long long actual_servo_rate;
static shared_ptr<ParamCallbacks> arm_param_cbs;
static shared_ptr<ParamCallbacks> head_param_cbs;
static shared_ptr<ControllerNG> arm_controller;
static shared_ptr<ControllerNG> head_controller;


static void usage(int ecode, std::string msg)
{
  errx(ecode,
       "%s\n"
       "  options:\n"
       "  -h               help (this message)\n"
       "  -v               verbose mode\n"
       "  -r  <filename>   arm specification (SAI XML format)\n"
       "  -c  <filename>   head specifiction (SAI XML format)\n"
       "  -f  <frequency>  servo rate (integer number in Hz, default 500Hz)\n"
       "  -s  <filename>   arm skill specification (YAML file with tasks etc)"
       "  -t  <filename>   head skill specification (YAML file with tasks etc)",
       msg.c_str());
}


static void parse_options(int argc, char ** argv)
{
  string arm_skill_spec("");
  string arm_spec("");
  string head_skill_spec("");
  string head_spec("");
  servo_rate = 500;
  
  for (int ii(1); ii < argc; ++ii) {
    if ((strlen(argv[ii]) < 2) || ('-' != argv[ii][0])) {
      usage(EXIT_FAILURE, "problem with option `" + string(argv[ii]) + "'");
    }
    else
      switch (argv[ii][1]) {
	
      case 'h':
	usage(EXIT_SUCCESS, "servo [-h] [-v] [-s skillspec] -r robotspec");
	
      case 'v':
	verbose = true;
 	break;
	
      case 'r':
 	++ii;
 	if (ii >= argc) {
	  usage(EXIT_FAILURE, "-r requires parameter");
 	}
	arm_spec = argv[ii];
 	break;

      case 'c':
	++ii;
	if (ii >= argc) {
	  usage(EXIT_FAILURE, "-c requires parameter");
	}
	head_spec = argv[ii];
	break;
	
      case 'f':
 	++ii;
 	if (ii >= argc) {
	  usage(EXIT_FAILURE, "-f requires parameter");
 	}
	else {
	  istringstream is(argv[ii]);
	  is >> servo_rate;
	  if ( ! is) {
	    usage(EXIT_FAILURE, "failed to read servo rate from `" + string(argv[ii]) + "'");
	  }
	  if (0 >= servo_rate) {
	    usage(EXIT_FAILURE, "servo rate has to be positive");
	  }
	}
 	break;
	
      case 's':
 	++ii;
 	if (ii >= argc) {
	  usage(EXIT_FAILURE, "-s requires parameter");
 	}
        arm_skill_spec = argv[ii];
 	break;

      case 't':
	++ii;
	if(ii >= argc) {
	  usage(EXIT_FAILURE, "-t requires parameter");
	}
	head_skill_spec = argv[ii];
	break;
	
      default:
	usage(EXIT_FAILURE, "invalid option `" + string(argv[ii]) + "'");
      }
  }
  
  try {
    if (arm_spec.empty() || head_spec.empty()) {
      usage(EXIT_FAILURE, "no robot specification (see option -r and -c)");
    }
    if (verbose) {
      warnx("reading arm spec from %s and head spec from %s", arm_spec.c_str(), head_spec.c_str());
    }
    static bool const enable_coriolis_centrifugal(false);
    arm_model.reset(jspace::test::parse_sai_xml_file(arm_spec, enable_coriolis_centrifugal));
    head_model.reset(jspace::test::parse_sai_xml_file(head_spec, enable_coriolis_centrifugal));
  }
  catch (runtime_error const & ee) {
    errx(EXIT_FAILURE,
	 "exception while parsing robot specification files\n"
	 "  filenames: %s %s\n"
	 "  error: %s",
	 arm_spec.c_str(), head_spec.c_str(), ee.what());
  }
  
  arm_factory.reset(new Factory());
  head_factory.reset(new Factory());
  
  Status st_arm;
  Status st_head;
  if (arm_skill_spec.empty() || head_skill_spec.empty()) {
    if (verbose) {
      usage(EXIT_FAILURE,"no arm and/or head skill file specified");
    }
  }
  else {
    if (verbose) {
      warnx("reading skills from %s and %s", arm_skill_spec.c_str(), head_skill_spec.c_str());
    }
    st_arm = arm_factory->parseFile(arm_skill_spec);
    st_head = head_factory->parseFile(head_skill_spec);
  }
  if ( ! st_arm) {
    errx(EXIT_FAILURE,
	 "failed to parse skills\n"
	 "  specification file: %s\n"
	 "  error description: %s",
	 arm_skill_spec.c_str(), st_arm.errstr.c_str());
  }
  if ( ! st_head) {
    errx(EXIT_FAILURE,
	 "failed to parse skills\n"
	 "  specification file: %s\n"
	 "  error description: %s",
	 head_skill_spec.c_str(), st_head.errstr.c_str());
  }
  if (verbose) {
    arm_factory->dump(cout, "*** parsed tasks and skills", "* ");
    head_factory->dump(cout, "*** parsed tasks and skills", "* ");
  }
}


static void handle(int signum)
{
  if (ros::ok()) {
    warnx("caught signal, requesting shutdown");
    ros::shutdown();
  }
  else {
    errx(EXIT_SUCCESS, "caught signal (again?), attempting forced exit");
  }
}


namespace {
  
  
  class Servo
    : public RTUtilDual
  {
  public:
    shared_ptr<Skill> arm_skill;    
    shared_ptr<Skill> head_skill;    
    
    virtual int init(jspace::State const & state1, jspace::State const & state2) {
      if (arm_skill || head_skill) {
	warnx("Servo::init(): already initialized");
	return -1;
      }
      if (arm_factory->getSkillTable().empty() || head_factory->getSkillTable().empty()) {
	warnx("Servo::init(): empty skill table");
	return -2;
      }
      if ( ! arm_model || ! head_model ) {
	warnx("Servo::init(): no model");
	return -3;
      }
      
      arm_model->update(state1);
      head_model->update(state2);
    
      jspace::Status status(arm_controller->init(*arm_model));
      if ( ! status) {
	warnx("Servo::init(): arm_controller->init() failed: %s", status.errstr.c_str());
	return -4;
      }
      status = head_controller->init(*head_model);
      if ( ! status) {
	warnx("Servo::init(): head_controller->init() failed: %s", status.errstr.c_str());
	return -4;
      }
      
      arm_skill = arm_factory->getSkillTable()[0]; // XXXX to do: allow selection at runtime
      status = arm_skill->init(*arm_model);
      if ( ! status) {
	warnx("Servo::init(): arm_skill->init() failed: %s", status.errstr.c_str());
	arm_skill.reset();
	return -5;
      }

      head_skill = head_factory->getSkillTable()[0]; // XXXX to do: allow selection at runtime
      status = head_skill->init(*head_model);
      if ( ! status) {
	warnx("Servo::init(): head_skill->init() failed: %s", status.errstr.c_str());
	head_skill.reset();
	return -5;
      }
      
      return 0;
    }
    
    
    virtual int update(jspace::State const & state1,
		       jspace::Vector & command1,
		       jspace::State const & state2,
		       jspace::Vector & command2)
    {
      if ( ! arm_skill || ! head_skill ) {
	warnx("Servo::update(): not initialized\n");
	return -1;
      }
      
      arm_model->update(state1);
      head_model->update(state2);
      
      jspace::Status status(arm_controller->computeCommand(*arm_model, *arm_skill, command1));
      if ( ! status) {
	warnx("Servo::update(): arm_controller->computeCommand() failed: %s", status.errstr.c_str());
	return -2;
      }

      status = head_controller->computeCommand(*head_model, *head_skill, command2);
      if ( ! status) {
	warnx("Servo::update(): head_controller->computeCommand() failed: %s", status.errstr.c_str());
	return -2;
      }
      
      return 0;
    }
    
    
    virtual int cleanup(void)
    {
      arm_skill.reset();
      head_skill.reset();
      return 0;
    }
    
    
    virtual int slowdown(long long iteration,
			 long long desired_ns,
			 long long actual_ns)
    {
      actual_servo_rate = 1000000000 / actual_ns;
      return 0;
    }
  };
  
}


int main(int argc, char ** argv)
{
  struct sigaction sa;
  bzero(&sa, sizeof(sa));
  sa.sa_handler = handle;
  if (0 != sigaction(SIGINT, &sa, 0)) {
    err(EXIT_FAILURE, "sigaction");
  }
  
  // Before we attempt to read any tasks and skills from the YAML
  // file, we need to inform the static type registry about custom
  // additions such as the HelloGoodbyeSkill.
  Factory::addSkillType<uta_opspace::HelloGoodbyeSkill>("uta_opspace::HelloGoodbyeSkill");
  Factory::addSkillType<uta_opspace::TaskPostureSkill>("uta_opspace::TaskPostureSkill");
  Factory::addSkillType<uta_opspace::WriteSkill>("uta_opspace::WriteSkill");
  Factory::addSkillType<uta_opspace::StaticAccuracyTest>("uta_opspace::StaticAccuracyTest");
  Factory::addSkillType<uta_opspace::DynamicAccuracyTest>("uta_opspace::DynamicAccuracyTest");
  Factory::addSkillType<uta_opspace::TrajAccuracyTest>("uta_opspace::TrajAccuracyTest");
  Factory::addSkillType<uta_opspace::CircleTest>("uta_opspace::CircleTest");
  Factory::addSkillType<uta_opspace::GestureSkill>("uta_opspace::GestureSkill");
  Factory::addSkillType<uta_opspace::JointTest>("uta_opspace::JointTest");
  
  
  ros::init(argc, argv, "wbc_m3_ctrl_servo", ros::init_options::NoSigintHandler);
  parse_options(argc, argv);
  ros::NodeHandle node("~");
  
  arm_controller.reset(new ControllerNG("wbc_m3_ctrl::servo arm"));
  head_controller.reset(new ControllerNG("wbc_m3_ctrl::servo head"));
  arm_param_cbs.reset(new ParamCallbacks());
  head_param_cbs.reset(new ParamCallbacks());
  Servo servo;
  try {
    if (verbose) {
      warnx("initializing param callbacks");
    }
    arm_registry.reset(arm_factory->createRegistry());
    arm_registry->add(arm_controller);
    arm_param_cbs->init(node, arm_registry, 1, 100);
    head_registry.reset(head_factory->createRegistry());
    head_registry->add(head_controller);
    head_param_cbs->init(node, head_registry, 1, 100);
    
    if (verbose) {
      warnx("starting servo with %lld Hz", servo_rate);
    }
    actual_servo_rate = servo_rate;
    servo.start(servo_rate);
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "failed to start servo: %s", ee.what());
  }
  
  warnx("started servo RT thread");
  ros::Time dbg_t0(ros::Time::now());
  ros::Time dump_t0(ros::Time::now());
  ros::Duration dbg_dt(0.1);
  ros::Duration dump_dt(0.05);
  
  while (ros::ok()) {
    ros::Time t1(ros::Time::now());
    if (verbose) {
      if (t1 - dbg_t0 > dbg_dt) {
	dbg_t0 = t1;
	servo.arm_skill->dbg(cout, "\n\n**************************************************", "");
	servo.head_skill->dbg(cout, "\n\n**************************************************", "");
	arm_controller->dbg(cout, "--------------------------------------------------", "");
	head_controller->dbg(cout, "--------------------------------------------------", "");
	cout << "--------------------------------------------------\n";
	jspace::pretty_print(arm_model->getState().position_, cout, "jpos", "  ");
	jspace::pretty_print(arm_model->getState().velocity_, cout, "jvel", "  ");
	jspace::pretty_print(arm_model->getState().force_, cout, "jforce", "  ");
	jspace::pretty_print(arm_controller->getCommand(), cout, "gamma", "  ");
	jspace::pretty_print(head_model->getState().position_, cout, "jpos", "  ");
	jspace::pretty_print(head_model->getState().velocity_, cout, "jvel", "  ");
	jspace::pretty_print(head_model->getState().force_, cout, "jforce", "  ");
	jspace::pretty_print(head_controller->getCommand(), cout, "gamma", "  ");
	Vector gravity;
	arm_model->getGravity(gravity);
	jspace::pretty_print(gravity, cout, "gravity", "  ");
	head_model->getGravity(gravity);
	jspace::pretty_print(gravity, cout, "gravity", "  ");
	cout << "servo rate: " << actual_servo_rate << "\n";
      }
    }
    if (t1 - dump_t0 > dump_dt) {
      dump_t0 = t1;
      arm_controller->qhlog(*servo.arm_skill, rt_get_cpu_time_ns() / 1000);
      head_controller->qhlog(*servo.head_skill, rt_get_cpu_time_ns() / 1000);
    }
    ros::spinOnce();
    usleep(10000);		// 100Hz-ish
  }
  
  warnx("shutting down");
  servo.shutdown();
}
