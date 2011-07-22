/*
 * Copyright (C) 2011 Roland Philippsen. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#include <opspace/Skill.hpp>
#include <opspace/Factory.hpp>
#include <opspace/parse_yaml.hpp>
#include <boost/scoped_ptr.hpp>
#include <stdexcept>
#include <err.h>
#include <jspace/test/sai_util.hpp>

using jspace::Status;
using opspace::Factory;
using namespace std;


int main(int argc, char ** argv)
{
  boost::scoped_ptr<jspace::Model> model;
  if (argc < 2) {
    errx(EXIT_FAILURE, "no skill file specified");
  }
  Factory factory;
  Status st;
  Factory::setDebugStream(&cout);
    try {
      st = factory.parseFile(argv[1]);
      if ( ! st) {
	std::cout << "ERROR parsing skill file `" << argv[1]
		  << "':\n  " << st.errstr << "\n";
      }
      else {
	std::cout << "skill file `" << argv[1] << "' PASSED\n";
      }
    }
    catch (YAML::Exception const & ee) {
      std::cout << "ERROR parsing skill file `" << argv[1]
		<< "':\n  unexpected YAML::Exception: " << ee.what() << "\n";
    }
    catch (std::runtime_error const & ee) {
      std::cout << "ERROR parsing skill file `" << argv[1]
		<< "':\n  unexpected std::runtime_error: " << ee.what() << "\n";
    }

  try {
    static bool const enable_coriolis_centrifugal(false);
    model.reset(jspace::test::parse_sai_xml_file(argv[2], enable_coriolis_centrifugal));
    std::cout << "model parsed\n";
  }
  catch (runtime_error const & ee) {
    errx(EXIT_FAILURE,
	 "exception while parsing robot specification\n");
  }

  boost::shared_ptr<opspace::Skill> skill;
  jspace::Status status;
  jspace::State state(7,7,7);
  model->update(state);

  try {
    for ( int i=0; i< factory.getSkillTable().size(); i++) {
      skill = factory.getSkillTable()[i];
      status = skill->init(*model);
      std::cout << "Skill " << i << " init\n";
	}
  }
  catch (std::runtime_error const & ee) {
    std::cout << "ERROR getting table " << ee.what() << "\n";
  } 

}
