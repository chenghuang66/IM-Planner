#pragma once

#include <queue>
#include <map>

//! Priority queue for integer coordinates with squared distances as priority.
/*
 * A priority queue that uses buckets to group elements with the same priority.
 *  The individual buckets are unsorted, which increases efficiency if these groups are large.
 *  The elements are assumed to be integer coordinates, and the priorities are assumed
 *  to be squared Euclidean distances (integers).
 */

template <typename T>
class BucketPrioQueue
{

public:
	//! Standard constructor
	/** Standard constructor. When called for the first time it creates a look up table
	 *  that maps square distances to bucket numbers, which might take some time...
	 */
	BucketPrioQueue();

	void clear()
	{ // 清空map
		buckets.clear();
		count = 0;
		nextPop = buckets.end();
	}

	//! Checks whether the Queue is empty
	bool empty();
	//! push an element
	void push(int prio, T t);
	//! return and pop the element with the lowest squared distance */
	T pop();

	int size() { return count; }
	int getNumBuckets() { return buckets.size(); }

	int getTopPriority()
	{
		return nextPop->first;
	}

private:
	int count;

	typedef std::map<int, std::queue<T>> BucketType;
	BucketType buckets;					   // map信息里，first是优先级信息（一般为几何距离的平方）， second是队列
	typename BucketType::iterator nextPop; // map的迭代器，指向优先级最高的元素
};

template <class T>
BucketPrioQueue<T>::BucketPrioQueue()
{
	clear();
}

template <class T>
bool BucketPrioQueue<T>::empty()
{
	return (count == 0);
}

template <class T>
void BucketPrioQueue<T>::push(int prio, T t)
{ // 添加元素
	buckets[prio].push(t);
	// 如果当前buckets为空或者优先级高于当前的nextPop,则将nextPop指向新插入元素
	if (nextPop == buckets.end() || prio < nextPop->first)
		nextPop = buckets.find(prio);
	count++;
}

template <class T>
T BucketPrioQueue<T>::pop()
{ // 弹出元素
	// 如果当前buckets不为空且nexePop指定优先级的队列为空，则nextPop为其上一个元素
	while (nextPop != buckets.end() && nextPop->second.empty())
		++nextPop;

	T p = nextPop->second.front(); // 从当前优先级队列里面获取front元素
	nextPop->second.pop();		   // 从当前优先级队列里面弹出front元素
	if (nextPop->second.empty())
	{ // 如果当前优先级队列为空了，则删除当前优先级队列
		typename BucketType::iterator it = nextPop;
		nextPop++;
		buckets.erase(it);
	}
	count--;
	return p;
}
