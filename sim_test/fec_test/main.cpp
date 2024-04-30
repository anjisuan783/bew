#include <stdint.h>
#include <vector>
#include <iostream>
#include <assert.h>
#include "test_func.h"

#include <iostream>
#include <vector>

using namespace std;

std::vector<vector<int>> g_result;

// 递归函数，生成组合
void generateCombinations(vector<int>& combination, int n, int k, int start) {
	// 如果已选择的数字个数等于k，输出组合并返回
	if (combination.size() == k) {
		//for (int num : combination) {
		//	cout << num << " ";
		//}
		//cout << endl;
		g_result.push_back(combination);
		return;
	}

	// 从start开始选择数字
	for (int i = start; i <= n; i++) {
		combination.push_back(i);  // 将当前数字加入组合
		generateCombinations(combination, n, k, i + 1);  // 递归选择下一个数字
		combination.pop_back();  // 移除最后一个数字，继续选择其他数字
	}
}

int main(int argc, const char* argv[])
{
	//test_fec_xor();
	//test_num_fec();
	//test_flex_sender(9);
	//test_flex_sender(80);
	//for(int protect = 0; protect < 10; ++protect)
	//	test_flex_receiver(protect, 1);
	//for (int protect = 10; protect < 256; ++protect)
	//	test_flex_receiver(protect, 5);

	int su_count = 0, fa_count = 0;
	int n = 21, k = 5; // C(22, 5)
	vector<int> combination;
	generateCombinations(combination, n, k, 0);

	static uint8_t loss[22] = {0x00};
	for (auto& x : g_result) {
		for (int i = 0; i < x.size(); ++i) {
			loss[i] = (uint8_t)x[i];
		}
		int ret = test_flex_receiver(10, x.size(), loss);
		//assert(ret == 0);
		ret == 0 ? ++su_count : ++fa_count;
	}
	float rate = 0;
	if (su_count + fa_count) {
		rate = (float)fa_count / (su_count + fa_count) * 100;
	}
	printf("total=%d, failed=%d rate=%.2f%%", su_count + fa_count, fa_count, rate);

	return 0;
}
