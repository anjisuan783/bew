#ifndef __SESSION_LOG_H__
#define __SESSION_LOG_H__

void ex_sim_log(int level, const char* file, int line, const char *fmt, ...);

#define sim_debug(...)  				ex_sim_log(0, __FILE__, __LINE__, __VA_ARGS__)
#define sim_info(...)    				ex_sim_log(1, __FILE__, __LINE__, __VA_ARGS__)
#define sim_warn(...)					ex_sim_log(2, __FILE__, __LINE__, __VA_ARGS__)
#define sim_error(...)   				ex_sim_log(3, __FILE__, __LINE__, __VA_ARGS__)

#define msg_log(addr, ...)						\
do{												\
	char ip[IP_SIZE] = { 0 };					\
	su_addr_to_string((addr), ip, IP_SIZE);		\
	sim_info(__VA_ARGS__);						\
} while (0)

#endif //!__SESSION_LOG_H__