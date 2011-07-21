#include "LetterManager.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace uta_opspace {

  Letter::Letter(std::string id, Vector points)
    :id_(id), points_(points) {}

  std::string Letter::id() { return id_; }
  Vector Letter::points() { return points_; }

  LetterManager::LetterManager(double threshold, double scale)
    : cur_row_(0),
      id_needs_init_(true),
      cur_letter_(-1),
      center_(0),
      threshold_(threshold),
      scale_(scale) {}

  //Loads the letters and their points from the provided file
  Status LetterManager::load(std::string file_name) {
    try {
      std::ifstream file(file_name.c_str());
      YAML::Parser parser(file);
      YAML::Node doc;
      parser.GetNextDocument(doc);
      std::string cur_letter;
      for (unsigned i=0; i<doc.size(); i++) {
	doc[i]["letter"] >> cur_letter;
	Vector points(Vector::Zero(doc[i]["points"].size()));
	for (unsigned j=0; j < doc[i]["points"].size(); j++) {
	  points[j] = doc[i]["points"][j];
	}
	all_letters_.push_back(new Letter(cur_letter,points));
      }
    } catch(YAML::ParserException& e) {
      return Status(false,e.what());
    }
    Status ok;
    return ok;
  }

  std::string LetterManager::id() {
    if ( !id_needs_init_ ) {
      return all_letters_[cur_letter_]->id();
    }
    return "";
}


  //Returns true if the current letter is complete
  //False otherwise
  //Needs to be updated to deal with the center and scaling
  //internally
  bool LetterManager::curPoint(Vector actual, Vector & point) {
   bool letterComplete = false;
   if( !id_needs_init_) {
     for (int i=0; i<3; i++) {
	 point[i] = all_letters_[cur_letter_]->points()[3*cur_row_+i];
       }
     Vector const delta( point - actual );
     double cur_dist = delta.norm();
     if ( cur_row_ < (all_letters_[cur_letter_]->points().rows()/3)-1 ) {
       if ( cur_dist < threshold_ ) {
	 ++cur_row_;
	 for (int i=0; i<3; i++) {
	   point[i] = all_letters_[cur_letter_]->points()[3*cur_row_+i];
	 }
       }
     }
     else { 
       letterComplete = true;
     }
   }
   else point = actual;
   return letterComplete;
}

  Status LetterManager::idIs(std::string id, Vector center) {

    //check if id in database
    //does not look for duplicates
    for (int i=0; i<all_letters_.size(); i++) {
      if (all_letters_[i]->id() == id) {
	id_needs_init_ = false;
	cur_row_ = 0;
	cur_letter_ = i;

	if ( center.rows() != 3) {
	    return Status(false,"Invalid or missing center vector");
	}
	center_ = center;

	Status ok;
	return ok;
      }
    }
    return Status(false,"ID not in database!"); 
  }

}
