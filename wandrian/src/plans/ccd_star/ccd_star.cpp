/*
 * ccd_star.cpp
 *
 *  Created on: Jul 20, 2016
 *      Author: manhnh
 */
#include "../../../include/plans/ccd_star/ccd_star.hpp"

namespace wandrian {
namespace plans {
namespace ccd_star {

CCDStar::CCDStar() :
    tool_size(0), cell_size(0), count(0) {
}

CCDStar::~CCDStar() {
}

void CCDStar::initialize(PointPtr starting_point, double tool_size) {
  path.clear();
  this->tool_size = tool_size;
  this->cell_size = tool_size / (2 * MR + 1);
  starting_cell = CellPtr(
      new Cell(PointPtr(new Point(starting_point->x, starting_point->y)),
          cell_size));
  starting_cell->set_overlapped(false);
  starting_cell->set_overlapped_r(false);
  starting_cell->set_visited(false);
  starting_cell->set_visited_r(false);
  starting_cell->set_parent(starting_cell);
  starting_cell->set_backpoint_d_star_extra(starting_cell);
  starting_cell->set_check_d_star_extra(false);
  list_cells.push_back(starting_cell);
  CellPtr temp_cell;
  int count_cell = 1;
  for (int i = 0; i < MAX_WiDTH_AND_HEIGHT; i++) {
    for (int j = 0; j < MAX_WiDTH_AND_HEIGHT; j++) {
      temp_cell = CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_point->x + i * cell_size,
                      starting_point->y + j * cell_size)), cell_size));
      if (temp_cell->get_center()->x == starting_point->x
          && temp_cell->get_center()->y == starting_point->y) {
        continue;
      } else {
        temp_cell->set_overlapped(false);
        temp_cell->set_overlapped_r(false);
        temp_cell->set_visited(false);
        temp_cell->set_visited_r(false);
        temp_cell->set_parent(temp_cell);
        temp_cell->set_backpoint_d_star_extra(temp_cell);
        temp_cell->set_check_d_star_extra(false);
        if (check_exist_obstacle(temp_cell) == false) {
          list_cells.push_back(temp_cell);
          count_cell++;
        }
      }
    }
    for (int j = 0; j < MAX_WiDTH_AND_HEIGHT; j++) {
      temp_cell = CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_point->x + i * cell_size,
                      starting_point->y - j * cell_size)), cell_size));
      if (temp_cell->get_center()->x == starting_point->x
          && temp_cell->get_center()->y == starting_point->y) {
        continue;
      } else {
        temp_cell->set_overlapped(false);
        temp_cell->set_overlapped_r(false);
        temp_cell->set_visited(false);
        temp_cell->set_visited_r(false);
        temp_cell->set_parent(temp_cell);
        temp_cell->set_backpoint_d_star_extra(temp_cell);
        temp_cell->set_check_d_star_extra(false);
        if (check_exist_obstacle(temp_cell) == false) {
          list_cells.push_back(temp_cell);
          count_cell++;
        }
      }
    }
    for (int j = 0; j < MAX_WiDTH_AND_HEIGHT; j++) {
      temp_cell = CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_point->x - i * cell_size,
                      starting_point->y + j * cell_size)), cell_size));
      if (temp_cell->get_center()->x == starting_point->x
          && temp_cell->get_center()->y == starting_point->y) {
        continue;
      } else {
        temp_cell->set_overlapped(false);
        temp_cell->set_overlapped_r(false);
        temp_cell->set_visited(false);
        temp_cell->set_visited_r(false);
        temp_cell->set_parent(temp_cell);
        temp_cell->set_backpoint_d_star_extra(temp_cell);
        temp_cell->set_check_d_star_extra(false);
        if (check_exist_obstacle(temp_cell) == false) {
          list_cells.push_back(temp_cell);
          count_cell++;
        }
      }
    }
    for (int j = 1; j < MAX_WiDTH_AND_HEIGHT; j++) {
      temp_cell = CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_point->x - i * cell_size,
                      starting_point->y - j * cell_size)), cell_size));
      if (temp_cell->get_center()->x == starting_point->x
          && temp_cell->get_center()->y == starting_point->y) {
        continue;
      } else {
        temp_cell->set_overlapped(false);
        temp_cell->set_overlapped_r(false);
        temp_cell->set_visited(false);
        temp_cell->set_visited_r(false);
        temp_cell->set_parent(temp_cell);
        temp_cell->set_backpoint_d_star_extra(temp_cell);
        temp_cell->set_check_d_star_extra(false);
        if (check_exist_obstacle(temp_cell) == false) {
          list_cells.push_back(temp_cell);
          count_cell++;
        }
      }
    }
  }
  std::cout << "Start Move, has " << count_cell << "\n";
//  sleep(2);
//  move_robot(starting_cell);
  path.insert(path.end(), starting_point);
}

bool CCDStar::check_exist_obstacle(CellPtr cell) {
  for (std::list<CellPtr>::iterator item = list_cells.begin();
      item != list_cells.end(); ++item) {
    if ((*item)->get_center()->x == cell->get_center()->x
        && (*item)->get_center()->y == cell->get_center()->y) {
      if ((*item)->get_cost_d_star() == INFINITY_COST
          && (*item)->get_cost_d_star_extra() == INFINITY_COST) {
        return true;
      }
    }
  }
  return false;
}

void CCDStar::scan(CellPtr current_cell) {
  std::cout << "Call D Star\n";
  d_star(current_cell, D_STAR, FIRST_CALL);
  std::cout << "End call D Star\n";
  for (std::list<CellPtr>::iterator next_cell = list_path.begin();
      next_cell != list_path.end(); ++next_cell) {

    if ((*next_cell)->get_center()->x == current_cell->get_center()->x
        && (*next_cell)->get_center()->y == current_cell->get_center()->y) {
      continue;
    } else {

      std::cout << "[Running]Current cell: " << current_cell->get_center()->x
          << " " << current_cell->get_center()->y << "\n";

      std::cout << "[Running]Current next cell: "
          << (*next_cell)->get_center()->x << " "
          << (*next_cell)->get_center()->y << " "
          << (*next_cell)->get_cost_d_star() << "\n";

      std::cout << "Start compute direction\n";
      VectorPtr direction = ((*next_cell)->get_center()
          - current_cell->get_center()) / tool_size;
      VectorPtr temp = ((*next_cell)->get_center() - current_cell->get_center())
          / tool_size;
      VectorPtr right_direction = ++temp;
      VectorPtr behind_direction = temp++;
      VectorPtr left_direction = temp++;
      std::cout << "Direction: " << "x: " << direction->x << " y: "
          << direction->y << "\n";
      std::cout << "right_direction: " << "x: " << right_direction->x << " y: "
          << right_direction->y << "\n";
      std::cout << "behind_direction: " << "x: " << behind_direction->x
          << " y: " << behind_direction->y << "\n";
      std::cout << "left_direction: " << "x: " << left_direction->x << " y: "
          << left_direction->y << "\n";
//      VectorPtr left_direction = direction--;
//      VectorPtr behind_direction = left_direction--;
//      VectorPtr right_direction = behind_direction--;
      std::cout << "End compute direction\n";

      // Check right
      std::cout << "Check right\n";
      if (see_obstacle(right_direction, tool_size / 2)) { // Obstacle
        std::cout << "Obstacle\n";
        //        d_star(current_cell, D_STAR, FIRST_CALL);
        CellPtr right_cell = CellPtr(
            new Cell(current_cell->get_center() + right_direction * cell_size,
                cell_size));

        if (!check_exist_obstacle(right_cell)) {
          for (std::list<CellPtr>::iterator item = list_cells.begin();
              item != list_cells.end(); ++item) {
            // Mark this cell is obstacle
            if ((*item)->get_center()->x == right_cell->get_center()->x
                && (*item)->get_center()->y == right_cell->get_center()->y) {
              (*item)->set_cost_d_star(INFINITY_COST);
              (*item)->set_cost_d_star_extra(
              INFINITY_COST);
            }
            // Mark other cell, which is not visited_r and overlapped_r is not visit and not overlapped
            // FIXME
//            if ((*item)->get_overlapped_r() == false) {
//              (*item)->set_overlapped(false);
//            }
//            if ((*item)->get_visited_r() == false) {
//              (*item)->set_visited(false);
//            }
          }
//          stop_robot();
////                  sleep(2);
//          scan(current_cell);
//          return;
        } else {
          // Go normal
        }
      }
      // End check right

      // Check behind
      std::cout << "Check behind\n";
      if (see_obstacle(behind_direction, tool_size / 2)) { // Obstacle
        std::cout << "Obstacle\n";
        //        d_star(current_cell, D_STAR, FIRST_CALL);
        CellPtr behind_cell = CellPtr(
            new Cell(current_cell->get_center() + behind_direction * cell_size,
                cell_size));

        if (!check_exist_obstacle(behind_cell)) {
          for (std::list<CellPtr>::iterator item = list_cells.begin();
              item != list_cells.end(); ++item) {
            // Mark this cell is obstacle
            if ((*item)->get_center()->x == behind_cell->get_center()->x
                && (*item)->get_center()->y == behind_cell->get_center()->y) {
              (*item)->set_cost_d_star(INFINITY_COST);
              (*item)->set_cost_d_star_extra(
              INFINITY_COST);
            }
            // Mark other cell, which is not visited_r and overlapped_r is not visit and not overlapped
            //FIXME
//            if ((*item)->get_overlapped_r() == false) {
//              (*item)->set_overlapped(false);
//            }
//            if ((*item)->get_visited_r() == false) {
//              (*item)->set_visited(false);
//            }
          }
//          stop_robot();
//          //        sleep(2);
//          scan(current_cell);
//          return;
        } else {
          // Go normal
        }
      }
      // End check behind

      // Check left
      std::cout << "Check left\n";
      if (see_obstacle(left_direction, tool_size / 2)) { // Obstacle
        std::cout << "Obstacle\n";
        //        d_star(current_cell, D_STAR, FIRST_CALL);
        CellPtr left_cell = CellPtr(
            new Cell(current_cell->get_center() + left_direction * cell_size,
                cell_size));

        if (!check_exist_obstacle(left_cell)) {
          for (std::list<CellPtr>::iterator item = list_cells.begin();
              item != list_cells.end(); ++item) {
            // Mark this cell is obstacle
            if ((*item)->get_center()->x == left_cell->get_center()->x
                && (*item)->get_center()->y == left_cell->get_center()->y) {
              (*item)->set_cost_d_star(INFINITY_COST);
              (*item)->set_cost_d_star_extra(
              INFINITY_COST);
            }
            // Mark other cell, which is not visited_r and overlapped_r is not visit and not overlapped
            //FIXME
//            if ((*item)->get_overlapped_r() == false) {
//              (*item)->set_overlapped(false);
//            }
//            if ((*item)->get_visited_r() == false) {
//              (*item)->set_visited(false);
//            }
          }
//          stop_robot();
//          //        sleep(2);
//          scan(current_cell);
//          return;
        } else {
          // Go normal
        }
      }
      // End check left
      // Check front
      std::cout << "Check front\n";
      if (see_obstacle(direction, tool_size / 2)) { // Obstacle
        std::cout << "Obstacle\n";
//        d_star(current_cell, D_STAR, FIRST_CALL);
//        if (!check_exist_obstacle((*next_cell))) {
        for (std::list<CellPtr>::iterator item = list_cells.begin();
            item != list_cells.end(); ++item) {
          // Mark this cell is obstacle
          if ((*item)->get_center()->x == (*next_cell)->get_center()->x
              && (*item)->get_center()->y == (*next_cell)->get_center()->y) {
            (*item)->set_cost_d_star(INFINITY_COST);
            (*item)->set_cost_d_star_extra(
            INFINITY_COST);
          }
          // Mark other cell, which is not visited_r and overlapped_r is not visit and not overlapped
          //FIXME
          if ((*item)->get_overlapped_r() == false) {
            (*item)->set_overlapped(false);
          }
          if ((*item)->get_visited_r() == false) {
            (*item)->set_visited(false);
          }
        }
        stop_robot();
//        sleep(2);
        std::cout
            << "----------------------------REPLANNING-----------------------\n";
        scan(current_cell);
        return;
//        } else{
//          // Go normal
//        }
      }
      // End check front

      // Go normal
      std::cout << "Go\n";

//        current_cell->set_visited_r(true);
//        current_cell->set_overlapped_r(true);

      for (std::list<CellPtr>::iterator item = list_cells.begin();
          item != list_cells.end(); ++item) {
        if ((*item)->get_center()->x == current_cell->get_center()->x
            && (*item)->get_center()->y == current_cell->get_center()->y) {
          (*item)->set_visited_r(true);
          (*item)->set_overlapped_r(true);
        }
      }

      go_with(direction, tool_size);
      current_cell = CellPtr(
          new Cell(
              PointPtr(
                  new Point((*next_cell)->get_center()->x,
                      (*next_cell)->get_center()->y)), cell_size));
      current_cell->set_cost_d_star((*next_cell)->get_cost_d_star());
      current_cell->set_cost_d_star_extra(
          (*next_cell)->get_cost_d_star_extra());
      current_cell->set_parent((*next_cell)->get_parent());
      current_cell->set_backpoint_d_star_extra(
          (*next_cell)->get_backpoint_d_star_extra());
      current_cell->set_check_d_star_extra(
          (*next_cell)->get_check_d_star_extra());
      current_cell->set_overlapped((*next_cell)->get_overlapped());
      current_cell->set_overlapped_r((*next_cell)->get_overlapped_r());
      current_cell->set_visited((*next_cell)->get_visited());
      current_cell->set_visited_r((*next_cell)->get_visited_r());

    }
  }
  std::cout << "Finish.\n";
  return;
}

void CCDStar::d_star(CellPtr current_cell, bool check_d_star,
    bool check_first_call) {

  std::cout << "Current cell: " << current_cell->get_center()->x << " "
      << current_cell->get_center()->y << " " << current_cell->get_cost_d_star()
      << "\n";

//  if (check_d_star == D_STAR) {
//    count++;
//    std::cout << "Loop " << count << " times\n";
//  }
  if (check_d_star == D_STAR && check_first_call == FIRST_CALL) {
    list_path.clear();
  } else if (check_d_star == D_STAR_EXTRA && check_first_call == FIRST_CALL) {
    list_path_d_star_extra.clear();
    for (std::list<CellPtr>::iterator item = list_cells.begin();
        item != list_cells.end(); ++item) {
      (*item)->set_check_d_star_extra(false);
    }
  }
  double tmp_cost;
  std::list<CellPtr> list_neighbors;
  CellPtr backpoint;
  // compute cost
  if (check_first_call == FIRST_CALL) {
    for (std::list<CellPtr>::iterator item = list_cells.begin();
        item != list_cells.end(); ++item) {
      if ((*item)->get_visited() == false
          && (*item)->get_overlapped() == false) {
        tmp_cost = sqrt(
            abs(
                (current_cell->get_center()->x - (*item)->get_center()->x)
                    * cell_size * 4
                    * (current_cell->get_center()->x - (*item)->get_center()->x)
                    * cell_size * 4)
                + abs(
                    (current_cell->get_center()->y - (*item)->get_center()->y)
                        * cell_size * 4
                        * (current_cell->get_center()->y
                            - (*item)->get_center()->y) * cell_size * 4));
      }
//      std::cout << tmp_cost << "\n";
      if (check_d_star == D_STAR) {
        if ((*item)->get_cost_d_star() != INFINITY_COST
            && (*item)->get_cost_d_star_extra() != INFINITY_COST) {
          (*item)->set_cost_d_star(tmp_cost);
        }
      } else if (check_d_star == D_STAR_EXTRA) {
        if ((*item)->get_cost_d_star() != INFINITY_COST
            && (*item)->get_cost_d_star_extra() != INFINITY_COST) {
          (*item)->set_cost_d_star_extra(tmp_cost);
        }
      }
    }
//    for (std::list<CellPtr>::iterator item = list_cells.begin();
//        item != list_cells.end(); ++item) {
//      std::cout << (*item)->get_cost_d_star() << "\n";
//    }
//    sleep(2);
  } else if (check_first_call == SECOND_CALL) {
//    std::cout << "Second call\n";
  }
// compute backpoint
//  for (;;) {
  CellPtr neighbor_up = CellPtr(
      new Cell(PointPtr(new Point(100, 100)), cell_size));
  CellPtr neighbor_down = CellPtr(
      new Cell(PointPtr(new Point(100, 100)), cell_size));
  CellPtr neighbor_left = CellPtr(
      new Cell(PointPtr(new Point(100, 100)), cell_size));
  CellPtr neighbor_right = CellPtr(
      new Cell(PointPtr(new Point(100, 100)), cell_size));
//  CellPtr neighbor_up, neighbor_down, neighbor_left, neighbor_right;
  for (std::list<CellPtr>::iterator item_for_check_neighbor =
      list_cells.begin(); item_for_check_neighbor != list_cells.end();
      ++item_for_check_neighbor) {
//      std::cout << "item cost: " << (*item_for_check_neighbor)->get_cost_d_star() << "\n";
    if ((*item_for_check_neighbor)->get_center()->x
        == current_cell->get_center()->x
        && (*item_for_check_neighbor)->get_center()->y
            == current_cell->get_center()->y + cell_size) {
//        neighbor_up = (*item_for_check_neighbor);
      neighbor_up = CellPtr(
          new Cell(
              PointPtr(
                  new Point((*item_for_check_neighbor)->get_center()->x,
                      (*item_for_check_neighbor)->get_center()->y)),
              cell_size));
      neighbor_up->set_cost_d_star(
          (*item_for_check_neighbor)->get_cost_d_star());
      neighbor_up->set_cost_d_star_extra(
          (*item_for_check_neighbor)->get_cost_d_star_extra());
      neighbor_up->set_parent((*item_for_check_neighbor)->get_parent());
      neighbor_up->set_backpoint_d_star_extra(
          (*item_for_check_neighbor)->get_backpoint_d_star_extra());
      neighbor_up->set_check_d_star_extra(
          (*item_for_check_neighbor)->get_check_d_star_extra());
      neighbor_up->set_overlapped((*item_for_check_neighbor)->get_overlapped());
      neighbor_up->set_overlapped_r(
          (*item_for_check_neighbor)->get_overlapped_r());
      neighbor_up->set_visited((*item_for_check_neighbor)->get_visited());
      neighbor_up->set_visited_r((*item_for_check_neighbor)->get_visited_r());
    }
    if ((*item_for_check_neighbor)->get_center()->x
        == current_cell->get_center()->x
        && (*item_for_check_neighbor)->get_center()->y
            == current_cell->get_center()->y - cell_size) {
//        neighbor_down = (*item_for_check_neighbor);
      neighbor_down = CellPtr(
          new Cell(
              PointPtr(
                  new Point((*item_for_check_neighbor)->get_center()->x,
                      (*item_for_check_neighbor)->get_center()->y)),
              cell_size));
      neighbor_down->set_cost_d_star(
          (*item_for_check_neighbor)->get_cost_d_star());
      neighbor_down->set_cost_d_star_extra(
          (*item_for_check_neighbor)->get_cost_d_star_extra());
      neighbor_down->set_parent((*item_for_check_neighbor)->get_parent());
      neighbor_down->set_backpoint_d_star_extra(
          (*item_for_check_neighbor)->get_backpoint_d_star_extra());
      neighbor_down->set_check_d_star_extra(
          (*item_for_check_neighbor)->get_check_d_star_extra());
      neighbor_down->set_overlapped(
          (*item_for_check_neighbor)->get_overlapped());
      neighbor_down->set_overlapped_r(
          (*item_for_check_neighbor)->get_overlapped_r());
      neighbor_down->set_visited((*item_for_check_neighbor)->get_visited());
      neighbor_down->set_visited_r((*item_for_check_neighbor)->get_visited_r());
    }
    if ((*item_for_check_neighbor)->get_center()->x
        == current_cell->get_center()->x - cell_size
        && (*item_for_check_neighbor)->get_center()->y
            == current_cell->get_center()->y) {
//        neighbor_left = (*item_for_check_neighbor);
      neighbor_left = CellPtr(
          new Cell(
              PointPtr(
                  new Point((*item_for_check_neighbor)->get_center()->x,
                      (*item_for_check_neighbor)->get_center()->y)),
              cell_size));
      neighbor_left->set_cost_d_star(
          (*item_for_check_neighbor)->get_cost_d_star());
      neighbor_left->set_cost_d_star_extra(
          (*item_for_check_neighbor)->get_cost_d_star_extra());
      neighbor_left->set_parent((*item_for_check_neighbor)->get_parent());
      neighbor_left->set_backpoint_d_star_extra(
          (*item_for_check_neighbor)->get_backpoint_d_star_extra());
      neighbor_left->set_check_d_star_extra(
          (*item_for_check_neighbor)->get_check_d_star_extra());
      neighbor_left->set_overlapped(
          (*item_for_check_neighbor)->get_overlapped());
      neighbor_left->set_overlapped_r(
          (*item_for_check_neighbor)->get_overlapped_r());
      neighbor_left->set_visited((*item_for_check_neighbor)->get_visited());
      neighbor_left->set_visited_r((*item_for_check_neighbor)->get_visited_r());
    }
    if ((*item_for_check_neighbor)->get_center()->x
        == current_cell->get_center()->x + cell_size
        && (*item_for_check_neighbor)->get_center()->y
            == current_cell->get_center()->y) {
//        neighbor_right = (*item_for_check_neighbor);
      neighbor_right = CellPtr(
          new Cell(
              PointPtr(
                  new Point((*item_for_check_neighbor)->get_center()->x,
                      (*item_for_check_neighbor)->get_center()->y)),
              cell_size));
      neighbor_right->set_cost_d_star(
          (*item_for_check_neighbor)->get_cost_d_star());
      neighbor_right->set_cost_d_star_extra(
          (*item_for_check_neighbor)->get_cost_d_star_extra());
      neighbor_right->set_parent((*item_for_check_neighbor)->get_parent());
      neighbor_right->set_backpoint_d_star_extra(
          (*item_for_check_neighbor)->get_backpoint_d_star_extra());
      neighbor_right->set_check_d_star_extra(
          (*item_for_check_neighbor)->get_check_d_star_extra());
      neighbor_right->set_overlapped(
          (*item_for_check_neighbor)->get_overlapped());
      neighbor_right->set_overlapped_r(
          (*item_for_check_neighbor)->get_overlapped_r());
      neighbor_right->set_visited((*item_for_check_neighbor)->get_visited());
      neighbor_right->set_visited_r(
          (*item_for_check_neighbor)->get_visited_r());
    }

    if ((*item_for_check_neighbor)->get_center()->x
        == current_cell->get_center()->x
        && (*item_for_check_neighbor)->get_center()->y
            == current_cell->get_center()->y) {
      //        neighbor_up = (*item_for_check_neighbor);
      (*item_for_check_neighbor)->set_overlapped(true);
      (*item_for_check_neighbor)->set_visited(true);
      std::cout << "Check current cell visited: x = "
          << current_cell->get_center()->x << " y = "
          << current_cell->get_center()->y << "\n";
    }

  }
//    std::cout << "Neighbor up: " << neighbor_up->get_center()->x << " "
//        << neighbor_up->get_center()->y << " " << neighbor_up->get_cost_d_star()
//        << "\nNeighbor down: " << neighbor_down->get_center()->x << " "
//        << neighbor_down->get_center()->y << " "
//        << neighbor_down->get_cost_d_star() << "\nNeighbor left: "
//        << neighbor_left->get_center()->x << " "
//        << neighbor_left->get_center()->y << " "
//        << neighbor_left->get_cost_d_star() << "\nNeighbor right: "
//        << neighbor_right->get_center()->x << " "
//        << neighbor_right->get_center()->y << " "
//        << neighbor_right->get_cost_d_star() << "\n";
  if (check_d_star == D_STAR) {
    list_neighbors.clear();
    if (neighbor_up->get_center()->x != 100) {
      if (neighbor_up->get_cost_d_star() != INFINITY_COST
          && neighbor_up->get_cost_d_star_extra() != INFINITY_COST
          && neighbor_up->get_visited() == false
          && neighbor_up->get_visited_r() == false) { // No obstacle and no visited
          //FIXME
//      if (neighbor_up->get_cost_d_star()
//          != INFINITY
//          && neighbor_up->get_cost_d_star_extra()
//              != INFINITY
//          && neighbor_up->get_visited() == false) { // No obstacle and no visited

        std::cout << "Neighbor up: " << neighbor_up->get_center()->x << " "
            << neighbor_up->get_center()->y << " "
            << neighbor_up->get_cost_d_star() << "\n";

        list_neighbors.push_back(neighbor_up);
      }
    }
    if (neighbor_down->get_center()->x != 100) {
      if (neighbor_down->get_cost_d_star() != INFINITY_COST
          && neighbor_down->get_cost_d_star_extra() != INFINITY_COST
          && neighbor_down->get_visited() == false
          && neighbor_down->get_visited_r() == false) { // No obstacle and no visited
          //FIXME
//      if (neighbor_down->get_cost_d_star()
//          != INFINITY
//          && neighbor_down->get_cost_d_star_extra()
//              != INFINITY
//          && neighbor_down->get_visited() == false) { // No obstacle and no visited

        std::cout << "Neighbor down: " << neighbor_down->get_center()->x << " "
            << neighbor_down->get_center()->y << " "
            << neighbor_down->get_cost_d_star() << "\n";

        list_neighbors.push_back(neighbor_down);
      }
    }
    if (neighbor_left->get_center()->x != 100) {
      if (neighbor_left->get_cost_d_star() != INFINITY_COST
          && neighbor_left->get_cost_d_star_extra() != INFINITY_COST
          && neighbor_left->get_visited() == false
          && neighbor_left->get_visited_r() == false) { // No obstacle and no visited
          //FIXME
//      if (neighbor_left->get_cost_d_star()
//          != INFINITY
//          && neighbor_left->get_cost_d_star_extra()
//              != INFINITY
//          && neighbor_left->get_visited() == false) { // No obstacle and no visited

        std::cout << "Neighbor left: " << neighbor_left->get_center()->x << " "
            << neighbor_left->get_center()->y << " "
            << neighbor_left->get_cost_d_star() << "\n";

        list_neighbors.push_back(neighbor_left);
      }
    }
    if (neighbor_right->get_center()->x != 100) {
      if (neighbor_right->get_cost_d_star() != INFINITY_COST
          && neighbor_right->get_cost_d_star_extra() != INFINITY_COST
          && neighbor_right->get_visited() == false
          && neighbor_right->get_visited_r() == false) { // No obstacle and no visited
          //FIXME
//      if (neighbor_right->get_cost_d_star()
//          != INFINITY
//          && neighbor_right->get_cost_d_star_extra()
//              != INFINITY
//          && neighbor_right->get_visited() == false) { // No obstacle and no visited

        std::cout << "Neighbor right: " << neighbor_right->get_center()->x
            << " " << neighbor_right->get_center()->y << " "
            << neighbor_right->get_cost_d_star() << "\n";

        list_neighbors.push_back(neighbor_right);
      }
    }
//    std::cout << "I am here." << list_neighbors.empty() << "\n";
    if (list_neighbors.empty() == true) {
      std::cout << "Call first call d star extra\n";
      d_star(current_cell, D_STAR_EXTRA, FIRST_CALL);   // Call D Star extra
//        break;
          //FIXME
      return;
    } else {
      double min_cost = -1;
      for (std::list<CellPtr>::iterator item_neighbor = list_neighbors.begin();
          item_neighbor != list_neighbors.end(); ++item_neighbor) {
        if (min_cost == -1 || min_cost > (*item_neighbor)->get_cost_d_star()) {
          min_cost = (*item_neighbor)->get_cost_d_star();
//            backpoint = (*item_neighbor);
          backpoint = CellPtr(
              new Cell(
                  PointPtr(
                      new Point((*item_neighbor)->get_center()->x,
                          (*item_neighbor)->get_center()->y)), cell_size));
          backpoint->set_cost_d_star((*item_neighbor)->get_cost_d_star());
          backpoint->set_cost_d_star_extra(
              (*item_neighbor)->get_cost_d_star_extra());
          backpoint->set_parent((*item_neighbor)->get_parent());
          backpoint->set_backpoint_d_star_extra(
              (*item_neighbor)->get_backpoint_d_star_extra());
          backpoint->set_check_d_star_extra(
              (*item_neighbor)->get_check_d_star_extra());
          backpoint->set_overlapped((*item_neighbor)->get_overlapped());
          backpoint->set_overlapped_r((*item_neighbor)->get_overlapped_r());
          backpoint->set_visited((*item_neighbor)->get_visited());
          backpoint->set_visited_r((*item_neighbor)->get_visited_r());
        }
      }
//      current_cell->set_parent(backpoint);
//      current_cell->set_overlapped(true);
//      current_cell->set_visited(true);
//TODO
      for (std::list<CellPtr>::iterator item = list_cells.begin();
          item != list_cells.end(); ++item) {
        if ((*item)->get_center()->x == current_cell->get_center()->x
            && (*item)->get_center()->y == current_cell->get_center()->y) {
          (*item)->set_parent(backpoint);
          (*item)->set_overlapped(true);
          (*item)->set_visited(true);
        }
      }

      list_path.push_back(backpoint);
      std::cout << "back point: " << backpoint->get_center()->x << " "
          << backpoint->get_center()->y << " " << backpoint->get_cost_d_star()
          << "\n";

      d_star(backpoint, D_STAR, SECOND_CALL);
//        break;
      return;
    }
  } else {
    //FIXME: D star extra
    std::cout << "Start d star extra\n";
//    current_cell->set_check_d_star_extra(true);

    for (std::list<CellPtr>::iterator item = list_cells.begin();
        item != list_cells.end(); ++item) {
      if ((*item)->get_center()->x == current_cell->get_center()->x
          && (*item)->get_center()->y == current_cell->get_center()->y) {
        (*item)->set_check_d_star_extra(true);
      }
    }

//    if (neighbor_up->get_cost_d_star() != INFINITY
//        && neighbor_up->get_cost_d_star_extra() != INFINITY
//        && neighbor_up->get_visited_r() == false) { // No obstacle and no visited with robot
    //FIXME
    if (neighbor_up->get_center()->x != 100) {
      if (neighbor_up->get_cost_d_star() != INFINITY_COST
          && neighbor_up->get_cost_d_star_extra() != INFINITY_COST) { // No obstacle and no visited with robot

        if (neighbor_up->get_visited() == false) {

          list_path_d_star_extra.push_back(neighbor_up);

          //TODO
          for (std::list<CellPtr>::iterator item_path =
              list_path_d_star_extra.begin();
              item_path != list_path_d_star_extra.end(); ++item_path) {
            list_path.push_back((*item_path));
          }
          list_path_d_star_extra.clear();

          d_star(neighbor_up, D_STAR, SECOND_CALL);
          return;
//          break;
        } else if (neighbor_up->get_check_d_star_extra() == false) {

          std::cout << "[D Star Extra]Neighbor up: "
              << neighbor_up->get_center()->x << " "
              << neighbor_up->get_center()->y << " "
              << neighbor_up->get_cost_d_star_extra() << "\n";

          list_neighbors.push_back(neighbor_up);
        }
      }
    }
//    if (neighbor_down->get_cost_d_star() != INFINITY
//        && neighbor_down->get_cost_d_star_extra() != INFINITY
//        && neighbor_down->get_visited_r() == false) { // No obstacle and no visited with robot
    //FIXME
    if (neighbor_down->get_center()->x != 100) {
      if (neighbor_down->get_cost_d_star() != INFINITY_COST
          && neighbor_down->get_cost_d_star_extra() != INFINITY_COST) { // No obstacle and no visited with robot
        if (neighbor_down->get_visited() == false) {

          list_path_d_star_extra.push_back(neighbor_down);

          //TODO
          for (std::list<CellPtr>::iterator item_path =
              list_path_d_star_extra.begin();
              item_path != list_path_d_star_extra.end(); ++item_path) {
            list_path.push_back((*item_path));
          }
          list_path_d_star_extra.clear();

          d_star(neighbor_down, D_STAR, SECOND_CALL);
          return;
//          break;
        } else if (neighbor_down->get_check_d_star_extra() == false) {

          std::cout << "[D Star Extra]Neighbor down: "
              << neighbor_down->get_center()->x << " "
              << neighbor_down->get_center()->y << " "
              << neighbor_down->get_cost_d_star_extra() << "\n";

          list_neighbors.push_back(neighbor_down);
        }
      }
    }
//    if (neighbor_left->get_cost_d_star() != INFINITY
//        && neighbor_left->get_cost_d_star_extra() != INFINITY
//        && neighbor_left->get_visited_r() == false) { // No obstacle and no visited with robot
    //FIXME
    if (neighbor_left->get_center()->x != 100) {
      if (neighbor_left->get_cost_d_star() != INFINITY_COST
          && neighbor_left->get_cost_d_star_extra() != INFINITY_COST) { // No obstacle and no visited with robot
        if (neighbor_left->get_visited() == false) {

          list_path_d_star_extra.push_back(neighbor_left);

          //TODO
          for (std::list<CellPtr>::iterator item_path =
              list_path_d_star_extra.begin();
              item_path != list_path_d_star_extra.end(); ++item_path) {
            list_path.push_back((*item_path));
          }
          list_path_d_star_extra.clear();

          d_star(neighbor_left, D_STAR, SECOND_CALL);
          return;
//          break;
        } else if (neighbor_left->get_check_d_star_extra() == false) {

          std::cout << "[D Star Extra]Neighbor left: "
              << neighbor_left->get_center()->x << " "
              << neighbor_left->get_center()->y << " "
              << neighbor_left->get_cost_d_star_extra() << "\n";

          list_neighbors.push_back(neighbor_left);
        }
      }
    }
//    if (neighbor_right->get_cost_d_star() != INFINITY
//        && neighbor_right->get_cost_d_star_extra() != INFINITY
//        && neighbor_right->get_visited_r() == false) { // No obstacle and no visited with robot
    //FIXME
    if (neighbor_right->get_center()->x != 100) {
      if (neighbor_right->get_cost_d_star() != INFINITY_COST
          && neighbor_right->get_cost_d_star_extra() != INFINITY_COST) { // No obstacle and no visited with robot
        if (neighbor_right->get_visited() == false) {

          list_path_d_star_extra.push_back(neighbor_right);

          //TODO
          for (std::list<CellPtr>::iterator item_path =
              list_path_d_star_extra.begin();
              item_path != list_path_d_star_extra.end(); ++item_path) {
            list_path.push_back((*item_path));
          }
          list_path_d_star_extra.clear();

          d_star(neighbor_right, D_STAR, SECOND_CALL);
          return;
//          break;
        } else if (neighbor_right->get_check_d_star_extra() == false) {

          std::cout << "[D Star Extra]Neighbor right: "
              << neighbor_right->get_center()->x << " "
              << neighbor_right->get_center()->y << " "
              << neighbor_right->get_cost_d_star_extra() << "\n";

          list_neighbors.push_back(neighbor_right);
        }
      }
    }
    if (list_neighbors.empty() == true) {
//        break;
      return;
    } else {
      double min_cost = -1;
      for (std::list<CellPtr>::iterator item_neighbor = list_neighbors.begin();
          item_neighbor != list_neighbors.end(); ++item_neighbor) {
        if (min_cost == -1
            || min_cost > (*item_neighbor)->get_cost_d_star_extra()) {
          min_cost = (*item_neighbor)->get_cost_d_star_extra();
//            backpoint = (*item_neighbor);
          backpoint = CellPtr(
              new Cell(
                  PointPtr(
                      new Point((*item_neighbor)->get_center()->x,
                          (*item_neighbor)->get_center()->y)), cell_size));
          backpoint->set_cost_d_star((*item_neighbor)->get_cost_d_star());
          backpoint->set_cost_d_star_extra(
              (*item_neighbor)->get_cost_d_star_extra());
          backpoint->set_parent((*item_neighbor)->get_parent());
          backpoint->set_backpoint_d_star_extra(
              (*item_neighbor)->get_backpoint_d_star_extra());
          backpoint->set_check_d_star_extra(
              (*item_neighbor)->get_check_d_star_extra());
          backpoint->set_overlapped((*item_neighbor)->get_overlapped());
          backpoint->set_overlapped_r((*item_neighbor)->get_overlapped_r());
          backpoint->set_visited((*item_neighbor)->get_visited());
          backpoint->set_visited_r((*item_neighbor)->get_visited_r());
        }
      }
//      current_cell->set_backpoint_d_star_extra(backpoint);
      for (std::list<CellPtr>::iterator item = list_cells.begin();
          item != list_cells.end(); ++item) {
        if ((*item)->get_center()->x == current_cell->get_center()->x
            && (*item)->get_center()->y == current_cell->get_center()->y) {
          (*item)->set_backpoint_d_star_extra(backpoint);
        }
      }
      //TODO
      list_path_d_star_extra.push_back(backpoint);
      d_star(backpoint, D_STAR_EXTRA, SECOND_CALL);
//        break;
      return;
    }
  }
//    break;
//  }
}

void CCDStar::set_behavior_see_obstacle(
    boost::function<bool(VectorPtr, double)> behavior_see_obstacle) {
  this->behavior_see_obstacle = behavior_see_obstacle;
}

void CCDStar::set_behavior_stop_robot(boost::function<void()> stop_robot) {
  this->stop_robot = stop_robot;
}

bool CCDStar::go_to(PointPtr position, bool flexibility) {
//  std::cout << "    pos: " << position->x << "," << position->y;
//  return BasePlan::go_to(position, flexibility);
  std::cout << "    pos: " << position->x << "," << position->y;
  path.insert(path.end(), position);
  if (behavior_go_to)
    return behavior_go_to(position, flexibility);
  return true;
}

bool CCDStar::see_obstacle(VectorPtr direction, double distance) {
  bool get_obstacle;
  if (behavior_see_obstacle) {
    get_obstacle = behavior_see_obstacle(direction, distance);
  } else {
    get_obstacle = false;
  }
  if (get_obstacle) {
    std::cout << "      \033[1;46m(OBSTACLE)\033[0m\n";
  }
  return get_obstacle;
}

void CCDStar::cover() {
  scan(starting_cell);
}

State CCDStar::state_of(CellPtr cell) {
//  State state = (old_cells.find(cell) != old_cells.end()) ? OLD : NEW;
//  if (state == OLD)
//    std::cout << " \033[1;45m(OLD)\033[0m\n";
  State state;
  if (cell->get_visited() == true && cell->get_overlapped() == true)
    state = OLD;
  else if (cell->get_visited() == false && cell->get_overlapped() == false)
    state = NEW;
  return state;
}

bool CCDStar::go_with(VectorPtr direction, double distance) {
  PointPtr last_position = path.back();
  PointPtr new_position = last_position + direction * distance;
  bool successful = go_to(new_position);
  std::cout << "\n";
  return successful;
}

}
}
}

