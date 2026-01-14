#pragma once

#include <limits>
#include <assert.h>
#include "point.hpp"

struct Grid
{
	enum GridState
	{
		origin, // 初始化
		ideal,	// 待更新
		free,	// 空闲
		occ,	// 占用
	};
	/* data */
	GridState sta; // 当前栅格状态
	/*
	一定要存储“栅格世界坐标"而非"栅格地图坐标"否则当Map Resize后无法通过：
	map.cell(Point)正确访问栅格！！！
	可通过如下程序测试：
		Point min(-100, -200);
		Point max(200, 300);
		Point nmin(-200, -300);
		Point nmax(300, 400);
		Point center((min.x + max.x) / 2.0, (min.y + max.y) / 2.0);
		DM map(center, min.x, min.y, max.x, max.y, 0.1);
		Point q1(66.9, 89.9);
		IntPoint q2 = map.world2map(q1);
		Point q3 = map.map2world(q2);
		map.cell(q2).setIdeal(q3);
		map.resize(nmin.x, nmin.y, nmax.x, nmax.y);
		IntPoint q4 = map.world2map(q1);
		Point q5 = map.map2world(q4);
		std::cout << q1 << " " << q2 << " " << q3  << " " << q4 << " " << q5 << std::endl;
		std::cout << map.cell(q1).pos << map.cell(q5).pos << std::endl;
		std::cout << map.cell(q2).pos << map.cell(q4).pos << std::endl;
	*/
	Point pos;	// 当前栅格世界坐标
	Point obs;	// 最近障碍物栅格世界坐标
	double dis; // 最近障碍物距离值
	Grid(double x_ = 0.0, double y_ = 0.0)
		: sta(origin), pos(x_, y_),
		  obs(std::numeric_limits<double>::max(),
			  std::numeric_limits<double>::max()),
		  dis(std::numeric_limits<double>::max()) {}
	inline GridState getSta() { return sta; }
	inline void setPos(double x_, double y_) { pos = Point(x_, y_); }
	inline void setPos(const Point &pos_) { pos = pos_; }
	inline void setObs(double x_, double y_) { obs = Point(x_, y_); }
	inline void setObs(const Point &obs_) { obs = obs_; }
	inline void setDis(double dis_) { dis = dis_; }
	inline void setIdeal()
	{
		setObs(std::numeric_limits<double>::max(),
			   std::numeric_limits<double>::max());
		setDis(std::numeric_limits<double>::max());
		sta = ideal;
	}
	inline void setIdeal(double posx_, double posy_)
	{
		setPos(posx_, posy_);
		setObs(std::numeric_limits<double>::max(),
			   std::numeric_limits<double>::max());
		setDis(std::numeric_limits<double>::max());
		sta = ideal;
	}
	inline void setIdeal(const Point &pos_) { setIdeal(pos_.x, pos_.y); }
	inline void setFree(double posx_, double posy_,
						double obsx_, double obsy_)
	{
		setPos(posx_, posy_);
		setObs(obsx_, obsy_);
		setDis(std::sqrt(std::pow(posx_ - obsx_, 2) +
						 std::pow(posy_ - obsy_, 2)));
		sta = free;
	}
	inline void setFree(const Point &pos_, const Point &obs_)
	{
		setFree(pos_.x, pos_.y, obs_.x, obs_.y);
	}
	inline void setOcc(double posx_, double posy_)
	{
		setPos(posx_, posy_);
		setObs(posx_, posy_);
		setDis(0.0);
		sta = occ;
	}
	inline void setOcc(const Point &pos_) { setOcc(pos_.x, pos_.y); }
};
typedef Grid::GridState Sta;

// GridAccumulator表示地图中一个cell（栅格）包括的内容
// GridAccumulator的一个对象，就是一个增强版栅格
/*
hits：     栅格被击中次数
visits：栅格被访问的次数
*/
struct GridAccumulator : public Grid
{
	// 构造函数
	GridAccumulator() : hits(0), visits(0) {}
	// 返回该栅格被占用的概率，范围是[0,1]
	inline operator double() const { return visits ? (double)hits / (double)visits : 0; }
	// 更新该栅格成员变量，value表示该栅格是否被击中，击中hits++,visits++,未击中仅visits++;
	inline void update(bool value)
	{
		if (value)
			hits++;
		visits++;
	}
	inline void update(int visits_, int hits_)
	{
		visits += visits_;
		hits += hits_;
	}
	// hits表示该栅格被击中的次数，visits表示该栅格被访问的次数
	int hits;
	int visits;
};
