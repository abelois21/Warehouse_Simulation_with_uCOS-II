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

									DISP_FGND_WHITE + DISP_BGND_LIGHT_GRAY,		// ��� ����� �� �� �Ͼ�� ǥ��
									DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY };	// ������ ���� ������ ǥ��
#define IDLE_COLOR DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY
#define STOP_COLOR DISP_FGND_WHITE + DISP_BGND_LIGHT_GRAY

void TaskCentralControl(void* pdata);	// �߾�ó��
void TaskRobotMove(void* pdata);		// 24��, �� �½�ũ�� ���� �ʿ��� �� �Ʒ� assignRoute�� ��û�� �� ���� 
void TaskGetOrder(void* pdata);			// ������ �޴� �Լ�, �켱���� ���� ����
void TaskAssigntRoute(void* pdata);		// �½�ũ�� �ڿ��� ���� ����� -> �� �ֱ⿡ ����� �� �ִ� ������ �Ǵ�.
									    // �޼����� ���� �ش� �κ����� �ִµ�, 1 ���� 2 ����
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

OS_EVENT* MOrder2Robot[N_ROBOT];			// ������ �޾�, �κ����� ������ �� ���Ǵ� ���Ϲڽ�
OS_EVENT* QRequestRoute;					// �κ��� ��θ� ��û�� �� �ʿ�
OS_EVENT* SemReceiveRoute[N_ROBOT];			// �κ��� ��θ� ��û�ϰ�, ���� ������ ����ϰ� ��
OS_EVENT* MutexRobotSync;					// �κ��� ��ġ������ �������ų�, ������ �� ����ȭ�� ����
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