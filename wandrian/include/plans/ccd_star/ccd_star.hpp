/*
 * ccd_star.hpp
 *
 *  Created on: Jul 20, 2016
 *      Author: manhnh
 */

#ifndef WANDRIAN_INCLUDE_PLANS_CCD_STAR_CCD_STAR_HPP_
#define WANDRIAN_INCLUDE_PLANS_CCD_STAR_CCD_STAR_HPP_

#define MAX_WiDTH_AND_HEIGHT 10 // (8*2+1)^2= 361 cell
#define MR 0
#define D_STAR true
#define D_STAR_EXTRA false
#define FIRST_CALL true
#define SECOND_CALL false

#include "../../common/vector.hpp"
#include "../../environment/cell.hpp"
#include "../base_plan.hpp"

using namespace wandrian::common;
using namespace wandrian::environment;

namespace wandrian {
namespace plans {
namespace ccd_star {

class CCDStar: public BasePlan {
public:
	CCDStar();
	~CCDStar();
	virtual void initialize(PointPtr, double);
//	virtual void move_robot(CellPtr);

	void set_behavior_see_obstacle(boost::function<bool(VectorPtr, double)>);
	void set_behavior_stop_robot(boost::function<void()>);
	void d_star(CellPtr, bool, bool);
	bool check_exist(CellPtr);
	int count;
  void cover();

protected:
	std::set<CellPtr, CellComp> old_cells;
	double tool_size; // = 'cell size' / 2
	double cell_size; // other_name: ecell, tool_size = (2*MR + 1)*ecell
	std::list<CellPtr> list_cells;
	std::list<CellPtr> list_path;
	bool go_to(PointPtr, bool = STRICTLY);
	bool see_obstacle(VectorPtr, double);
	virtual void scan(CellPtr);
	virtual State state_of(CellPtr);

private:
	CellPtr starting_cell;
	boost::function<bool(VectorPtr, double)> behavior_see_obstacle;
	boost::function<void()> stop_robot;
	bool go_with(VectorPtr, double);
};

typedef boost::shared_ptr<CCDStar> CCDStarPtr;

}
}
}

#endif /* WANDRIAN_INCLUDE_PLANS_CCD_STAR_CCD_STAR_HPP_ */
