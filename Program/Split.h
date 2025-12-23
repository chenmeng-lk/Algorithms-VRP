/*MIT License

Copyright(c) 2020 Thibaut Vidal

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef SPLIT_H
#define SPLIT_H

#include "Params.h"
#include "Individual.h"

struct ClientSplit
{
	double demand;
	double serviceTime;
	double d0_x;// 从车场(0)到客户x的距离
	double dx_0;//从客户x到车场(0)的距离
	double dnext;//从客户x到下一个客户(x+1)的距离
	ClientSplit() : demand(0.), serviceTime(0.), d0_x(0.), dx_0(0.), dnext(0.) {};
};

// Simple Deque which is used for all Linear Split algorithms
struct Trivial_Deque
{
	std::vector <int> myDeque; // Simply a vector structure to keep the elements of the queue
	int indexFront; // Index of the front element
	int indexBack; // Index of the back element
	inline void pop_front(){indexFront++;} // Removes the front element of the queue D
	inline void pop_back(){indexBack--;} // Removes the back element of the queue D
	inline void push_back(int i){indexBack++; myDeque[indexBack] = i;} // Appends a new element to the back of the queue D
	inline int get_front(){return myDeque[indexFront];}
	inline int get_next_front(){return myDeque[indexFront + 1];}
	inline int get_back(){return myDeque[indexBack];}
	void reset(int firstNode) { myDeque[0] = firstNode; indexBack = 0; indexFront = 0; }
	inline int size(){return indexBack - indexFront + 1;}
	
	Trivial_Deque(int nbElements, int firstNode)
	{
		myDeque = std::vector <int>(nbElements);
		myDeque[0] = firstNode;
		indexBack = 0;
		indexFront = 0;
	}
};

class Split
{

 private:

 // Problem parameters
 const Params & params ;
 int maxVehicles ;

 /* Auxiliary data structures to run the Linear Split algorithm */
 std::vector < ClientSplit > cliSplit;
 std::vector < std::vector < double > > potential;  // Potential vector potential[k][i] = 使用k辆车服务前i个客户的最小惩罚成本
 std::vector < std::vector < int > > pred;  // Indice of the predecessor in an optimal path 达到potential[k][i]时，前一辆车服务的最后一个客户索引
 std::vector <double> sumDistance; // sumDistance[i] for i > 1 contains the sum of distances : sum_{k=1}^{i-1} d_{k,k+1} 前i个客户之间的连续距离和
 std::vector <double> sumLoad; // sumLoad[i] for i >= 1 contains the sum of loads : sum_{k=1}^{i} q_k 前i个客户的总需求
 std::vector <double> sumService; // sumService[i] for i >= 1 contains the sum of service time : sum_{k=1}^{i} s_k 前i个客户的总服务时间

 // To be called with i < j only
 // Computes the cost of propagating the label i until j
 //已使用k辆车服务前i个客户，再计算到第j个客户的成本
 inline double propagate(int i, int j, int k)
 {//i已经被上一辆车服务了，故从i+1开始
	 return potential[k][i] + sumDistance[j] - sumDistance[i + 1] + cliSplit[i + 1].d0_x + cliSplit[j].dx_0
		 + params.penaltyCapacity * std::max<double>(sumLoad[j] - sumLoad[i] - params.vehicleCapacity, 0.);
 }

 // Tests if i dominates j as a predecessor for all nodes x >= j+1
 // We assume that i < j
 inline bool dominates(int i, int j, int k)
 {//对于所有x >= j+1，从i转移都比从j转移更优
	 return potential[k][j] + cliSplit[j + 1].d0_x > potential[k][i] + cliSplit[i + 1].d0_x + sumDistance[j + 1] - sumDistance[i + 1]
		 + params.penaltyCapacity * (sumLoad[j] - sumLoad[i]);
 }//从j开始创建新路线到x的成本 > 从i开始创建新路线到x的成本

 // Tests if j dominates i as a predecessor for all nodes x >= j+1
 // We assume that i < j 检查节点j是否从右侧支配节点i
 inline bool dominatesRight(int i, int j, int k)
 {
	 return potential[k][j] + cliSplit[j + 1].d0_x < potential[k][i] + cliSplit[i + 1].d0_x + sumDistance[j + 1] - sumDistance[i + 1] + MY_EPSILON;
 }

  // Split for unlimited fleet
  int splitSimple(Individual & indiv);

  // Split for limited fleet
  int splitLF(Individual & indiv);

public:

  // General Split function (tests the unlimited fleet, and only if it does not produce a feasible solution, runs the Split algorithm for limited fleet)
  void generalSplit(Individual & indiv, int nbMaxVehicles);

  // Constructor
  Split(const Params & params);

};
#endif
