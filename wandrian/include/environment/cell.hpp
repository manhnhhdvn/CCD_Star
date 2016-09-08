/*
 * cell.hpp
 *
 *  Created on: Mar 11, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_CELL_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_CELL_HPP_

#include "../common/rectangle.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {

enum State {
  NEW, OLD, OBSTACLE
};

class Cell: public Rectangle {

public:
  Cell(PointPtr, double);
  virtual ~Cell();

  double get_size() const;
  boost::shared_ptr<Cell> get_parent();
  void set_parent(boost::shared_ptr<Cell>);
  boost::shared_ptr<Cell> get_backpoint_d_star_extra();
  void set_backpoint_d_star_extra(boost::shared_ptr<Cell>);

  bool get_overlapped() const;
  void set_overlapped(bool);
  bool get_overlapped_r() const;
  void set_overlapped_r(bool);
  bool get_visited() const;
  void set_visited(bool);
  bool get_visited_r() const;
  void set_visited_r(bool);
  double get_cost_d_star() const;
  void set_cost_d_star(double cost_d_star);
  double get_cost_d_star_extra() const;
  void set_cost_d_star_extra(double cost_d_star_extra);

  bool get_check_d_star_extra() const;
  void set_check_d_star_extra(bool);

//  int get_name() const;
//  void set_name(int);

private:
  bool visited_r;
  bool visited;
  bool overlapped_r;
  bool overlapped;
  double cost_d_star;
  double cost_d_star_extra;
  bool check_d_star_extra;
  boost::shared_ptr<Cell> parent;
  boost::shared_ptr<Cell> backpoint_d_star_extra;
//  int name;
};

typedef boost::shared_ptr<Cell> CellPtr;
typedef boost::shared_ptr<Cell const> CellConstPtr;

inline bool operator<(CellConstPtr c1, CellConstPtr c2) {
  return
      std::abs(c1->get_center()->x - c2->get_center()->x) > EPSILON ?
          (c1->get_center()->x < c2->get_center()->x) :
          (std::abs(c1->get_center()->y - c2->get_center()->y) > EPSILON ?
              (c1->get_center()->y < c2->get_center()->y) :
              c1->get_size() < c2->get_size());
}

inline bool operator!=(CellPtr c1, CellPtr c2) {
  return c1 < c2 || c2 < c1 || c1->get_size() != c2->get_size();
}

inline bool operator==(CellPtr c1, CellPtr c2) {
  return !(c1 != c2);
}

struct CellComp {
  bool operator()(CellConstPtr c1, CellConstPtr c2) const {
    return c1 < c2;
  }
};

}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_CELL_HPP_ */
