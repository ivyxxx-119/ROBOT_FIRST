#ifndef ROBOT_API_HPP
#define ROBOT_API_HPP

/*
 * C/C++共享接口。
 * 虽然后缀是hpp，内容仍然兼容C。
 */
#ifdef __cplusplus
extern "C" {
#endif

void Robot_Init(void);
void Robot_Task(void);

#ifdef __cplusplus
}
#endif

#endif
