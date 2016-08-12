/*
 * cell.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: cslab
 */

#include "../../include/environment/cell.hpp"

namespace wandrian {
namespace environment {

Cell::Cell(PointPtr center, double size) :
    cost_d_star(std::numeric_limits<double>::infinity()), cost_d_star_extra(
        std::numeric_limits<double>::infinity()), overlapped(false), overlapped_r(
        false), visited(false), visited_r(false), check_d_star_extra(false), Rectangle(
        center, size, size) {
}

Cell::~Cell() {
}

double Cell::get_size() const {
  return (get_width() + get_height()) / 2;
}

CellPtr Cell::get_parent() {
  return parent;
}

void Cell::set_parent(CellPtr parent) {
  this->parent = parent;
}

CellPtr Cell::get_backpoint_d_star_extra() {
  return parent;
}

void Cell::set_backpoint_d_star_extra(CellPtr parent) {
  this->parent = parent;
}

bool Cell::get_overlapped() const {
  return overlapped;
}

void Cell::set_overlapped(bool overlapped) {
  this->overlapped = overlapped;
}

bool Cell::get_overlapped_r() const {
  return overlapped_r;
}

void Cell::set_overlapped_r(bool overlapped_r) {
  this->overlapped_r = overlapped_r;
}

bool Cell::get_visited() const {
  return visited;
}

void Cell::set_visited(bool visited) {
  this->visited = visited;
}

bool Cell::get_visited_r() const {
  return visited_r;
}

void Cell::set_visited_r(bool visited_r) {
  this->visited_r = visited_r;
}

double Cell::get_cost_d_star() const {
  return cost_d_star;
}

void Cell::set_cost_d_star(double cost_d_star) {
  this->cost_d_star = cost_d_star;
}

double Cell::get_cost_d_star_extra() const {
  return cost_d_star_extra;
}

void Cell::set_cost_d_star_extra(double cost_d_star_extra) {
  this->cost_d_star_extra = cost_d_star_extra;
}

bool Cell::get_check_d_star_extra() const {
  return check_d_star_extra;
}

void Cell::set_check_d_star_extra(bool check_D_Star_Extra) {
  this->check_d_star_extra = check_D_Star_Extra;
}

//int Cell::get_name() const {
//  return name;
//}
//
//void Cell::set_name(int name) {
//  this->name = name;
//}

}
}
