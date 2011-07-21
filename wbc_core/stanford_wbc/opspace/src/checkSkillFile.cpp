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
#include <stdexcept>
#include <err.h>

using jspace::Status;
using opspace::Factory;
using namespace std;


int main(int argc, char ** argv)
{
  if (argc < 2) {
    errx(EXIT_FAILURE, "no skill file specified");
  }
  
  Factory::setDebugStream(&cout);
  for (int ii(1); ii < argc; ++ii) {
    try {
      Factory factory;
      Status st;
      st = factory.parseFile(argv[ii]);
      if ( ! st) {
	std::cout << "ERROR parsing skill file `" << argv[ii]
		  << "':\n  " << st.errstr << "\n";
      }
      else {
	std::cout << "skill file `" << argv[ii] << "' PASSED\n";
      }
    }
    catch (YAML::Exception const & ee) {
      std::cout << "ERROR parsing skill file `" << argv[ii]
		<< "':\n  unexpected YAML::Exception: " << ee.what() << "\n";
    }
    catch (std::runtime_error const & ee) {
      std::cout << "ERROR parsing skill file `" << argv[ii]
		<< "':\n  unexpected std::runtime_error: " << ee.what() << "\n";
    }
  }
}
