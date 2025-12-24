#ifndef CIRCLESECTOR_H
#define CIRCLESECTOR_H

// Simple data structure to represent circle sectors
// Angles are measured in [0,65535] instead of [0,359], in such a way that modulo operations are much faster (since 2^16 = 65536)
// Credit to Fabian Giesen at "https://web.archive.org/web/20200912191950/https://fgiesen.wordpress.com/2015/09/24/intervals-in-modular-arithmetic/" for useful implementation tips regarding interval overlaps in modular arithmetics 
//
// （总体说明）：
// 该结构用于表示围绕中心点的一段极角扇区（区间可跨越 0 点），角度采用 16 位模空间 [0,65535] 来表示。
// 选择 2^16 的好处：模运算可通过位运算得到高效实现（编译器通常将 %65536 优化为 &0xffff），并且在区间包含/重叠判定时
// 可用正模差值来快速比较两段区间的长度与相对位置，从而避免复杂的条件分支。
struct CircleSector
{
	int start;
	int end;

	// Positive modulo 65536
	static int positive_mod(int i)
	{
		// 1) Using the formula positive_mod(n,x) = (n % x + x) % x
		// 2) Moreover, remark that "n % 65536" should be automatically compiled in an optimized form as "n & 0xffff" for faster calculations
		return (i % 65536 + 65536) % 65536;
	}

	/*
	（positive_mod）:
	- 计算对 65536 的正模，保证返回值在 [0,65535]。
	- 通过 (i % M + M) % M 的形式避免负数模导致负值的问题。
	- 由于 M 为 2 的幂，编译器可能把模操作优化为按位与，从而加速计算。
	*/

	// Initialize a circle sector from a single point
	void initialize(int point)
	{
		start = point;
		end = point;
	}

	/*
	（initialize）:
	- 用单个角度点初始化扇区，使得扇区起止点相同，表示包含该点的最小区间。
	*/

	// Tests if a point is enclosed in the circle sector
	bool isEnclosed(int point)
	{
		return (positive_mod(point - start) <= positive_mod(end - start));
	}

	/*
	（isEnclosed）:
	- 判断给定点（以同样的 16 位角度表示）是否落在当前扇区内。
	- 逻辑说明：把点与 start 的差以及 end 与 start 的差都取正模，若前者不超过后者则视为包含。
	- 该方法对跨越 0 点的区间也能正确工作，因为使用了模差的比较。
	*/

	// Tests overlap of two circle sectors
	static bool overlap(const CircleSector & sector1, const CircleSector & sector2)
	{
		return ((positive_mod(sector2.start - sector1.start) <= positive_mod(sector1.end - sector1.start))
			|| (positive_mod(sector1.start - sector2.start) <= positive_mod(sector2.end - sector2.start)));
	}

	/*
	（overlap）:
	- 测试两段扇区是否存在交集。实现思想为：若 sector2 的起点落在 sector1 的包含区间内，则两者重叠；同理检查反向情况。
	- 采用正模差比较即可正确处理跨 0 点的环绕情形（例如 [65000, 100] 与 [50,200]）。
	*/

	// Extends the circle sector to include an additional point 
	// Done in a "greedy" way, such that the resulting circle sector is the smallest
	void extend(int point)
	{
		if (!isEnclosed(point))
		{
			if (positive_mod(point - end) <= positive_mod(start - point))
				end = point;
			else
				start = point;
		}
	}

	/*
	（extend）:
	- 将一个新点纳入当前扇区，采用贪心策略以保持扇区尽可能小。
	- 逻辑：若点已在扇区内则不变；否则比较把点扩展到现有 end 处与扩展到 start 处哪种扩展导致的新增弧长更短，选择较短的那一侧扩展。
	- 使用模差比较可以统一处理跨 0 点情形。
	*/
};

#endif
