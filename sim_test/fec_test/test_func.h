#ifndef __test_func_h_
#define __test_func_h_

#ifdef __cplusplus
extern "C" {
#endif
	void test_fec_xor();
	void test_num_fec();
	void test_flex_sender(uint8_t protect_fraction);
	int test_flex_receiver(uint8_t protect_fraction, int loss_num, uint8_t[]);

#ifdef __cplusplus
}
#endif

#endif



