#include <jspace/wrap_eigen.hpp>
#include <jspace/Status.hpp>
#include <vector>

using jspace::Vector;
using jspace::Matrix;
using jspace::Status;

namespace uta_opspace {

  class Letter {
  private:
    std::string id_;
    Vector points_;
  public:
    Letter(std::string id, Vector points);
    std::string id();
    Vector points();
  };

class LetterManager {

private:
  int cur_row_;
  double threshold_;
  double scale_;
  bool id_needs_init_;
  std::vector<Letter *> all_letters_;
  int cur_letter_;
  Vector center_;

public:
  LetterManager(double threshold, double scale); 
  Status load(std::string file_name);
  Status idIs(std::string id, Vector center);
  std::string id();
  bool curPoint(Vector actual, Vector & point);  
};

}
