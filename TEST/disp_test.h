#ifndef _DISP_TEST_H_
#define _DISP_TEST_H_

#include "includes.h"
#include "disp_test.h"
#include "my_util.h"
#include <time.h>
#define N_ROBOT			 16
#define N_COLOR			  6
#define N_SHELF	     	196
#define N_PLACE		      9
#define TASK_STK_SIZE	512
#define TASK_PRIO        16

const INT8U Colors[N_COLOR + 2] = { DISP_FGND_RED + DISP_BGND_LIGHT_GRAY,
									DISP_FGND_BLUE + DISP_BGND_LIGHT_GRAY,
									DISP_FGND_GREEN + DISP_BGND_LIGHT_GRAY,
									DISP_FGND_YELLOW + DISP_BGND_LIGHT_GRAY,
									DISP_FGND_CYAN + DISP_BGND_LIGHT_GRAY,
									DISP_FGND_GRAY + DISP_BGND_LIGHT_GRAY,

									DISP_FGND_WHITE + DISP_BGND_LIGHT_GRAY,		// 경로 재생성 일 때 하얀색 표시
									DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY };	// 정지된 상태 검은색 표시
#define IDLE_COLOR DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY
#define STOP_COLOR DISP_FGND_WHITE + DISP_BGND_LIGHT_GRAY

void TaskCentralControl(void* pdata);	// 중앙처리
void TaskRobotMove(void* pdata);		// 24개, 이 태스크는 길이 필요할 시 아래 assignRoute에 요청할 수 있음 
void TaskGetOrder(void* pdata);			// 오더를 받는 함수, 우선순위 가장 낮음
void TaskAssigntRoute(void* pdata);		// 태스크가 자원을 많이 사용함 -> 한 주기에 사용할 수 있는 제한을 건다.
									    // 메세지에 따라 해당 로봇에게 주는데, 1 전역 2 지역
static void TaskStartDispInit();						
static void TaskViewClear();
void TaskViewDisp();

/*
__inline Pos Loc2Pos(INT16U loc);
__inline INT16U Pos2Loc(Pos pos);
__inline INT16U frontOfShelf(INT16U shelf_no, INT16U shelf_loc);
__inline INT16U originalShelf(INT16U shelf_loc);
*/

int uint2str(char* strp, int n);
int binary2str(char* strp, INT16U n, int length);
void debug_print(INT8U row, const char* format, ...);

OS_EVENT* MOrder2Robot[N_ROBOT];			// 오더를 받아, 로봇에게 전달할 떄 사용되는 메일박스
OS_EVENT* QRequestRoute;					// 로봇이 경로를 요청할 떄 필요
OS_EVENT* SemReceiveRoute[N_ROBOT];			// 로봇이 경로를 요청하고, 받을 떄까지 대기하게 함
OS_EVENT* MutexRobotSync;					// 로봇의 위치정보를 가져오거나, 수정할 때 동기화를 위함
OS_EVENT* SemWorkSpace[2];
OS_STK TaskStk[4][TASK_STK_SIZE];
OS_STK TaskStk2[N_ROBOT][TASK_STK_SIZE];

Pos ShelfPos[N_SHELF];
Pos TPPos[N_PLACE];
Pos LDPos[N_PLACE];
Pos ParkingPos[N_ROBOT];
INT8U blockedPos[HEIGHT][WIDTH];

RobotInfo robots[N_ROBOT];

INT32U idle_shelf[N_SHELF / 32 + 1];
INT16U idle_tpp;
INT16U idle_ldp;
INT16U idle_park;

__inline struct Pos Loc2Pos(INT16U loc)
{
	struct Pos pos;
	pos.i = (loc & 0xFF) - POS_TOP;
	pos.j = ((loc >> 8) - POS_LEFT) / 2;

	return pos;
}

__inline INT16U Pos2Loc(Pos pos)
{
	return (INT16U)(((pos.j * 2) + POS_LEFT) << 8 | (pos.i + POS_TOP));
}

__inline INT16U frontOfShelf(INT16U shelf_no, INT16U shelf_loc)
{
	if (shelf_no % 2 == 0) {
		return (shelf_loc - (2 << 8));
	}
	else {
		return (shelf_loc + (2 << 8));
	}
}

__inline INT16U originalShelf(INT16U shelf_loc)
{
	INT8U m = (shelf_loc >> 8) % 10;
	if (m == 0) {
		return shelf_loc - (2 << 8);
	}
	else if (m == 6) {
		return shelf_loc + (2 << 8);
	}
	else {
		return -1;
	}

}

#endif // _DISP_TEST_H_