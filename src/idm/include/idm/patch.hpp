#pragma once

#include <memory>
#include <set>
#include "point.hpp"

template <typename Cell>
class Patch
{
protected:
    // 二维栅格的的尺寸
    int xsize, ysize;

public:
    Cell **cells;
    // 构造函数
    Patch(int xsize_, int ysize_)
        : xsize(xsize_), ysize(ysize_)
    {
        // std::cout << "Patch Construction Function X Size: " << xsize_ << std::endl;
        // std::cout << "Patch Construction Function Y Size: " << ysize_ << std::endl;
        cells = new Cell *[xsize];
        for (int x = 0; x < xsize; ++x)
        {
            cells[x] = new Cell[ysize];
        }
        // std::cout << "Patch Construction Finish" << std::endl;
    }
    // 析构函数
    ~Patch()
    {
        if (cells)
        {
            for (int i = 0; i < xsize; ++i)
                delete[] cells[i];
            delete[] cells;
        }
    }
    // 判断某个栅格是否在该栅格地图内
    inline bool isInside(int x, int y) const
    {
        return x >= 0 && x < xsize && y >= 0 && y < ysize;
    }
    inline bool isInside(const IntPoint &p) const { return isInside(p.x, p.y); }
    // 输入栅格坐标（x，y）返回对应的栅格对象
    inline Cell &cell(int x, int y)
    {
        assert(isInside(x, y));
        return cells[x][y];
    }
    inline Cell &cell(const IntPoint &p) { return cell(p.x, p.y); }
    inline const Cell &cell(int x, int y) const
    {
        assert(isInside(x, y));
        return cells[x][y];
    }
    inline const Cell &cell(const IntPoint &p) const { return cell(p.x, p.y); }
    // 获取二维栅格的XY尺寸
    inline int getXSize() const { return xsize; }
    inline int getYSize() const { return ysize; }
};

// 枚举类型
enum AccessibilityState
{
    Outside = 0x0,
    Inside = 0x1,
    Allocated = 0x2
};

template <typename Cell>
class Hpatch : public Patch<std::shared_ptr<Patch<Cell>>>
{
public:
    typedef std::set<point<int>, pointcomparator<int>> PointSet;

protected:
    // 为一个地图补丁中的小栅格申请内存
    virtual Patch<Cell> *createPatch() const
    {
        return new Patch<Cell>(1 << patchMagnitude, 1 << patchMagnitude);
    }
    PointSet activeArea; // 存储地图中使用到的Cell的坐标
    int patchMagnitude;  // patch的大小等级
    int patchSize;       // patch的实际大小
public:
    // 构造函数
    Hpatch(int xsize_, int ysize_, int patchMagnitude_ = 3)
        : Patch<std::shared_ptr<Patch<Cell>>>::Patch((xsize_ >> patchMagnitude_), (ysize_ >> patchMagnitude_))
    {
        patchMagnitude = patchMagnitude_; // 地图补丁的大小等级
        patchSize = 1 << patchMagnitude_; // 每块地图补丁的边大小，而非每块地图补丁中的栅格数目
    }
    // 析构函数
    virtual ~Hpatch() {}
    // 调整存储 “地图补丁” 的二维数组的大小，也就是个数变化
    void resize(int xmin, int ymin, int xmax, int ymax);
    // 输入 “栅格” 的栅格坐标做入口参数，返回 “地图补丁” 的 “栅格坐标”，也就是一个栅格属于哪个地图补丁
    inline IntPoint patchIndexes(int x, int y) const
    {
        if (x >= 0 && y >= 0)
            return IntPoint(x >> patchMagnitude, y >> patchMagnitude);
        return IntPoint(-1, -1);
    }
    inline IntPoint patchIndexes(const IntPoint &p) const { return patchIndexes(p.x, p.y); }
    // 输入 “栅格” 栅格坐标做入口参数，返回该 “地图补丁” 的内存是否已经分配
    inline bool isAllocated(int x, int y) const
    {
        IntPoint c = patchIndexes(x, y); // 转换到大栅格地图补丁patch的坐标
        std::shared_ptr<Patch<Cell>> &ptr = this->cells[c.x][c.y];
        return (ptr != nullptr);
    }
    inline bool isAllocated(const IntPoint &p) const { return isAllocated(p.x, p.y); }
    // 输入 “栅格” 栅格坐标做入口参数，返回一个栅格对象
    inline Cell &cell(int x, int y)
    {
        IntPoint c = patchIndexes(x, y);
        assert(this->isInside(c.x, c.y)); // “地图补丁” 的 “栅格坐标” 是否在整个存储“地图补丁”的二维数组中
        if (!this->cells[c.x][c.y])       // 若指向为空，说明还未申请则为其申请内存空间
        {
            Patch<Cell> *patch = createPatch();                          // 为每一块地图补丁patch申请内存
            this->cells[c.x][c.y] = std::shared_ptr<Patch<Cell>>(patch); // 该地图补丁指向了此时的patch指针，不再为空了
        }
        std::shared_ptr<Patch<Cell>> &ptr = this->cells[c.x][c.y];
        return (*ptr).cell(IntPoint(x - (c.x << patchMagnitude), y - (c.y << patchMagnitude)));
    }
    inline Cell &cell(const IntPoint &p) { return cell(p.x, p.y); }
    inline const Cell &cell(int x, int y) const
    {
        assert(isAllocated(x, y));
        IntPoint c = patchIndexes(x, y);
        const std::shared_ptr<Patch<Cell>> &ptr = this->cells[c.x][c.y];
        return (*ptr).cell(IntPoint(x - (c.x << patchMagnitude), y - (c.y << patchMagnitude)));
    }
    inline const Cell &cell(const IntPoint &p) const { return cell(p.x, p.y); }
    // 输入 “栅格” 栅格坐标做入口参数，返回“地图补丁”的状态
    inline AccessibilityState cellState(int x, int y) const
    {
        if (this->isInside(patchIndexes(x, y)))
        {
            if (isAllocated(x, y))
                return (AccessibilityState)((int)Inside | (int)Allocated);
            else
                return Inside;
        }
        return Outside;
    }
    inline AccessibilityState cellState(const IntPoint &p) const { return cellState(p.x, p.y); }
    // 设置地图的有效区域
    inline void setActiveArea(const PointSet &aa, bool patchCoords = false)
    {
        activeArea.clear();
        for (PointSet::const_iterator it = aa.begin(); it != aa.end(); ++it)
        {
            IntPoint p;
            if (patchCoords) // 是否是patch的坐标
                p = *it;
            else
                p = patchIndexes(*it);
            activeArea.insert(p); // 将一个个地图补丁大栅格坐标插入m_activeArea中，元素唯一，自动排序
        }
    }
    // 给有效区域(局部地图or被激光扫过的区域)分配内存
    inline void allocActiveArea()
    {
        for (PointSet::const_iterator it = activeArea.begin(); it != activeArea.end(); ++it) // 遍历地图补丁大栅格
        {
            // std::cout << "Patch Index: " << it->x << ", " << it->y << std::endl;
            const std::shared_ptr<Patch<Cell>> &ptr = this->cells[it->x][it->y];
            Patch<Cell> *patch = nullptr;
            // 如果对应的active没有被分配内存 则进行内存分配
            // 一个patch的内存没有分配的话，是没有内存存储栅格的，也就是没有Patch的对象的二级指针
            if (!ptr)
            {
                patch = createPatch(); // 分配指向小栅格的内存
            }
            else
            {
                patch = new Patch<Cell>(*ptr);
            }
            this->cells[it->x][it->y] = std::shared_ptr<Patch<Cell>>(patch); // 每一个地铺补丁大栅格都指向了一个分配了的地图补丁，存储每一个小栅格
        }
    }
    // 基本函数
    inline int getPatchSize() const { return patchMagnitude; }
    inline int getPatchMagnitude() const { return patchMagnitude; }
};

template <typename Cell>
void Hpatch<Cell>::resize(int xmin, int ymin, int xmax, int ymax)
{
    // 新地图补丁数组的尺寸
    int cols = xmax - xmin;
    int rows = ymax - ymin;
    // 为新的“地图补丁”二维数组申请内存
    std::shared_ptr<Patch<Cell>> **newcells(new std::shared_ptr<Patch<Cell>> *[cols]);
    for (int x = 0; x < cols; x++)
    {
        newcells[x] = new std::shared_ptr<Patch<Cell>>[rows];
        for (int y = 0; y < rows; y++)
        {
            newcells[x][y] = std::shared_ptr<Patch<Cell>>(nullptr); // 指向为NULL
        }
    }
    // xmin、ymin最小值为0，因为地图坐标没有负值的缘故
    int dx = xmin < 0 ? 0 : xmin;
    int dy = ymin < 0 ? 0 : ymin;
    // 此方法可以合理的遍历内存，不越界也不做无用的访问，因为只取原来的值
    int Dx = xmax < this->xsize ? xmax : this->xsize;
    int Dy = ymax < this->ysize ? ymax : this->ysize;
    // 把原来的地图中的栅格数据赋值到新的内存中，然后销毁原来的指针
    for (int x = dx; x < Dx; ++x)
    {
        for (int y = dy; y < Dy; ++y)
        {
            newcells[x - xmin][y - ymin] = this->cells[x][y];
        }
        delete[] this->cells[x]; // 销毁每一列，从小向大销毁
    }
    delete[] this->cells;
    this->cells = newcells; // 新的二级指针指向老的二级指针
    this->xsize = cols;
    this->ysize = rows;
}
