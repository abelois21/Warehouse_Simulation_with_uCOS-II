#include "includes.h"
#include "disp_test.h"
#include "my_util.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>


int main(void)
{
	INT8U i;
	void* QBuff[N_ROBOT+1];

	OSInit();

	// initialize idle codes
	idle_tpp    = 0b111111111;
	idle_ldp    = 0b111111111;
	idle_park   = 0;	// 모든 로봇이 주차된 상태
	for (i = 0; i < N_SHELF / 32 + 1; i++) 
		idle_shelf[i] = -1;
	
	// initialize task
	OSTaskCreate(TaskCentralControl, (void *)0, &TaskStk[0][TASK_STK_SIZE - 1], TASK_PRIO - 1);
	OSTaskCreate(TaskGetOrder,		 (void *)0, &TaskStk[1][TASK_STK_SIZE - 1], TASK_PRIO + N_ROBOT + 1);
	OSTaskCreate(TaskGetOrder,		 (void *)0, &TaskStk[2][TASK_STK_SIZE - 1], TASK_PRIO + N_ROBOT + 2);
	OSTaskCreate(TaskAssigntRoute,   (void *)0, &TaskStk[3][TASK_STK_SIZE - 1], TASK_PRIO);

	for (i = 1; i < N_ROBOT+1; i++) {
//		OSTaskCreate(TaskRobotMove,  (void *)0, &TaskStk[i][TASK_STK_SIZE - 1], TASK_PRIO + i);
		SemReceiveRoute[i] = OSSemCreate(0);
	}

	QRequestRoute  = OSQCreate(QBuff, N_ROBOT + 1);
	MutexRobotSync = OSSemCreate(1);

	OSStart();

	return 0;
}

void TaskCentralControl(void* pdata)
{
	INT8U msg[40];
	INT16S key;

	TaskStartDispInit();

	for (;;) {
		if (PC_GetKey(&key)) {                             /* if key has been pressed              */
			if (key == 0x1B) {                             /* if it's the ESCAPE key               */
				exit(0);                                   /* return to OS                         */
			}
		}

		// *******************************   test   *******************************
//		RouteRequestInfo rri;
//		rri.start_pos
//		request = *((RouteRequestInfo*)OSQPend(QRequestRoute, 0, &ERR));	// 로봇으로 부터 경로할당 요청 수신

		// *******************************   test   *******************************


		PC_GetDateTime(msg);
		PC_DispStr(78, 22, msg, DISP_FGND_YELLOW + DISP_BGND_BLUE);	// 

		TaskViewDisp();

		OSTimeDly(1);
	}
}

/*********************************************************************************************************
*	robot related futions
*********************************************************************************************************/

void TaskRobotMove(void* pdata)
{ 
	const INT8U robot_no = OSTCBCur->OSTCBPrio % N_ROBOT;
	const INT8U d_value = 3;
	INT8U i, j, tmp_color, ERR, park = -1, isBlocked = 0;
	INT16U cur_step, max_step, tmp_step = 0, tmp_route_cnt = -1;

	const Pos* ParkingPos_ptr = setParkingPos(ParkingPos, N_ROBOT);

	struct RobotInfo* cur_robot = &robots[robot_no];
	cur_robot->pos = ParkingPos_ptr[robot_no];	// 로봇의 초기 위치 설정
	cur_robot->stat = IDLE;
	INT8U park_no = robot_no;

	struct Pos* cur_route;
	struct Pos global_route[70], local_route[20];

	struct OrderInfo order;		// 오더를 준 태스크에서 오더는 계속 바뀌니까, 포인터로 하면 안될듯
								// 로봇의 우선순위를 높게 해서, 새 오더를 생성하는 시점보다 빠르게 받기
	struct RouteRequestInfo request_info, t_request_info;
	request_info.robot_no = robot_no; t_request_info.robot_no = robot_no;

	MOrder2Robot[robot_no] = OSMboxCreate((void *)0);	// mailbox to receive order

	while (1) {
		order = *((OrderInfo *)OSMboxPend(MOrder2Robot[robot_no], 0, &ERR)); // 로봇에 오더가 전달될 때 까지 대기
		cur_robot->stat = RUNNING;

		for (i = 0; i < 3; i++) {
			// request info initialization
			request_info.start_pos = cur_robot->pos;

			for (i = 0; i < 70; i++) {
				global_route[i].i = 0; 
				global_route[i].j = 0;
			}
			cur_route = global_route;
			request_info.route = cur_route;

			max_step = 0;
			request_info.max_step = &max_step;
			request_info.robot_no = robot_no;
			request_info.kind = GLOBAL_ROUTE;

			switch (i) {
			case 0:
				request_info.arrival_pos = Loc2Pos(order.start_loc);
				idle_park |= (1 << park_no);
				break;
			case 1:
				request_info.arrival_pos = Loc2Pos(order.arrival_loc);
				break;
			case 2:
				INT16U t, min = -1;
				for (j = 0; j < N_ROBOT; j++) {	// 현재 로봇의 위치로 부터 가장 가까운 주차 위치
					if (!idle_park & (1 << j)) continue;

					t = distance(cur_robot->pos, ParkingPos_ptr[j]);
					if (min > t) {
						min = t;
						park_no = j;
					}
					
				}
				idle_park -= (1 << park_no);
				request_info.arrival_pos = ParkingPos_ptr[park_no];
				break;
			}
			
			OSQPost(QRequestRoute, &request_info);					// 목적지까지의 경로 요청
			OSSemPend(SemReceiveRoute[robot_no], 0, &ERR);		    // 경로 받음

			// 경로를 따라 전진
			while (1) {		
				if ((request_info.arrival_pos.i == cur_robot->pos.i) 
					&& (request_info.arrival_pos.j == cur_robot->pos.j)) {		// 목적지에 도착
					OSSemPost(SemWorkSpace[order.kind]);				        // 작업장 세마포어 방출
					break;
				}

				// 전방에 다른 로봇이 있는지 탐색
				// ****************** 뮤텍스 로봇 위치정보 보호 ******************
				OSMutexPend(MutexRobotSync, 0, &ERR);
				for (j = 0, isBlocked = 0; j < N_ROBOT; j++) {	
					if (j == robot_no) continue;
					if (robots[j].pos.i == cur_robot->pos.i && robots[j].pos.j == cur_robot->pos.j) {
						isBlocked = 1;
						break;
					}
				}
				OSMutexPost(MutexRobotSync);
				// ****************** 뮤텍스 로봇 위치정보 보호 ******************

				// 로봇이 경로를 따라 진행 중 가로막혔을 때
				if (isBlocked) {	
					tmp_color = cur_robot->color;
					cur_robot->stat = STOPED;
					cur_robot->color = STOP_COLOR;

					OSTimeDly(1);

					// 지역 경로 요청 t_rri에 담아서, route와 을 가져옴
					t_request_info.start_pos = cur_robot->pos;
					if (max_step - cur_step < d_value) 
						t_request_info.arrival_pos = request_info.arrival_pos;
					else 
						t_request_info.arrival_pos = global_route[cur_step + d_value];
				
					for (j = 0; j < 20; j++) {
						local_route[j].i = 0;
						local_route[j].j = 0;
					}
					cur_route = local_route;
					t_request_info.route = local_route;

					t_request_info.max_step = &tmp_route_cnt;
					t_request_info.robot_no = robot_no;
					t_request_info.kind = LOCAL_ROUTE;

					// 경로 요청 완료. -> 메일박스로 받음
					OSQPost(QRequestRoute, &t_request_info);					// 목적지까지의 경로 요청
					OSSemPend(SemReceiveRoute[robot_no], 0, &ERR);		// 경로 받음

					cur_robot->stat = RUNNING;
					cur_robot->color = tmp_color;

					tmp_step  = cur_step;
					cur_step  = 0;

					continue;
				}

				if (tmp_route_cnt > 0 && tmp_route_cnt == 1) {	// 지역 경로 상 목적지 도착 -> 전역 경로로 돌아가기
					cur_step  = tmp_step + d_value;
					cur_route = global_route + cur_step;
					tmp_route_cnt = -1;
				}

				// 로봇 경로를 따라 한 칸 이동
				// ****************** 뮤텍스 로봇 위치정보 보호 ******************
				OSMutexPend(MutexRobotSync, 0, &ERR);
				cur_robot->pos.i = (cur_route + cur_step)->i;
				cur_robot->pos.i = (cur_route + cur_step)->j;
				cur_step += 1;
				OSMutexPost(MutexRobotSync);
				// ****************** 뮤텍스 로봇 위치정보 보호 ******************

				OSTimeDly(1);
			}
		}

		// 로봇의 작업이 끝난 후 로봇의 상태 및 장업장의 상태를 유휴상태로 되돌리기.

		cur_robot->stat = IDLE;
		cur_robot->color = IDLE_COLOR;

		if (order.kind == SHELF2WS) {
			i = (order.arrival_loc >> 8) / 8;
			idle_tpp |= (1 << i);

			PC_DispStr(originalShelf(order.start_loc) >> 8, originalShelf(order.start_loc) & 0xFF, "■", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
			PC_DispStr(2 + 8 * i, 1, "|      |", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
		}
		else if (order.kind == WS2SHELF) {
			i = (order.start_loc >> 8) / 8;
			idle_ldp |= (1 << i);

			PC_DispStr(originalShelf(order.arrival_loc) >> 8, originalShelf(order.arrival_loc) & 0xFF, "■", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
			PC_DispStr(2 + 8 * i, 23, "|      |", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
		}
		
		OSTimeDly(1);
	}
}

/*********************************************************************************************************
*	find route related futions
*********************************************************************************************************/

void TaskAssigntRoute(void* pdata) 
{
	INT8U ERR, cnt = 4;
	RouteRequestInfo request;

	setblockedPos(blockedPos, ShelfPos, N_SHELF);
	const INT8U(*blocked_ptr)[WIDTH] = blockedPos;

	while (1) {
//!!!!!		request = *((RouteRequestInfo*)OSQPend(QRequestRoute, 0, &ERR));	// 로봇으로 부터 경로할당 요청 수신

		AStarSearch(request.route, blocked_ptr, request.max_step, request.start_pos, request.arrival_pos);
//!!!!!	OSSemPost(SemReceiveRoute[request.robot_no]);	// 할당 완료

		if (request.kind == GLOBAL_ROUTE) OSTimeDly(1);
		else if (request.kind == LOCAL_ROUTE) {			// 지역 경로 요청
			if (--cnt) {
				cnt = 4;;
				OSTimeDly(1);
			}
		}
	}
}

/*********************************************************************************************************
*	order related futions
*********************************************************************************************************/

void TaskGetOrder(void* pdata) 
{
	INT8U i, idx, ERR, kind, shelf, robotm, place, robot, tmp, min = -1, random;
	INT16U shelf_loc;
	Pos t_pos;
	OrderInfo order;

	const Pos* ShelfPos_ptr = setShelfPos(ShelfPos);
	const Pos* TPPos_ptr    = setTPPos(TPPos, N_PLACE);
	const Pos* LDPos_ptr    = setLDPos(LDPos, N_PLACE);
	const RobotInfo* robots_ptr = robots;
	
	kind = OSTCBCur->OSTCBPrio % 2 + 1;
	order.kind = kind;
	INT16U* const idle_place = (kind == SHELF2WS) ? &idle_tpp : &idle_ldp;
	
	SemWorkSpace[kind] = OSSemCreate(N_PLACE);
	*idle_place = INITIAL_PLACE;

	srand(time(0) + (OSTCBCur->OSTCBPrio * 237 >> 4));

	while (1) {
		OSSemPend(SemWorkSpace[kind], 0, &ERR);			// check that all places are busy

		idx = (INT8U)(rand() % N_PLACE);

//		debug_print(7, "idx: %d code: %b ", idx, *idle_place);	// ************test************

		for (i = 0; i < N_PLACE; i++, idx++) {
			if (idx == N_PLACE) idx = 0;
			if ((*idle_place) & (1 << idx)) {
				place = idx;
				(*idle_place) = (*idle_place) - (1 << idx);
				break;
			}
		}

//		debug_print(8, "place: %d code: %b ", place, *idle_place);	// ************test************

		
//		debug_print(9, "shelf: %d code: %b ", shelf, idle_shelf);	// ************test************

		shelf = getIdleShelf(idle_shelf, N_SHELF, (INT8U)(rand() % N_SHELF));
		shelf_loc = Pos2Loc(ShelfPos_ptr[shelf]);

		order.color = Colors[(INT8U)(rand() % N_COLOR)];

//		debug_print(10, "shelf: %d code: %b ", shelf, idle_shelf);	// ************test************

/*
		if (kind == SHELF2WS)											// warehouse to work place
		{	
			t_pos = TPPos_ptr[place];
			order.start_loc   = frontOfShelf(shelf, shelf_loc);
			order.arrival_loc = Pos2Loc(t_pos);

			PC_DispStr(2 + 8 * place, 1, "*", order.color);		// update display
			PC_DispStr(2 + 8 * (place + 1) - 1, 1, "*", order.color);		// update display
		}
		else if (kind == WS2SHELF)										// work place to warehouse 
		{
			t_pos = LDPos_ptr[place];
			order.start_loc   = Pos2Loc(t_pos);
			order.arrival_loc = frontOfShelf(shelf, shelf_loc);

			PC_DispStr(2 + 8 * place, 23, "*", order.color);		// update display
			PC_DispStr(2 + 8 * (place + 1) - 1, 23, "*", order.color);		// update display
		}
*/

		PC_DispStr(2 + 8 * place, 1 + (kind / 2 * 22), "*", order.color);					// update display
		PC_DispStr(2 + 8 * (place + 1) - 1, 1 + (kind / 2 * 22), "*", order.color);			// update display
		PC_DispStr((INT8U)(shelf_loc >> 8), (INT8U)(shelf_loc & 0xFF), "★", order.color);	// update display

		t_pos             = (kind == SHELF2WS) ? TPPos_ptr[place] : LDPos_ptr[place];
		order.start_loc   = (kind == SHELF2WS) ? frontOfShelf(shelf, shelf_loc) : Pos2Loc(t_pos);
		order.arrival_loc = (kind == SHELF2WS) ? Pos2Loc(t_pos) : frontOfShelf(shelf, shelf_loc);


		// find out robot which is most close to start point.
		for (i = 0; i < N_ROBOT; i++) {
			if (robots_ptr->stat == IDLE) {
				tmp = distance(t_pos, robots_ptr->pos);
				if (min > tmp) {
					min = tmp;
					robot = i;
				}
			}
		}
		min = -1;	// 최댓값으로 갱신

		debug_print(11, "getOrder from %d to %d robot %d", place, shelf, robot);	// ************test************
		debug_print(4, "the number of orders: %d", ++numOfOrder);	// ************test************

		OSMboxPost(MOrder2Robot[robot], &order);	// send order to the robot task by the robot's mailbox

		OSTimeDly((INT8U)(rand() % 10 + 15));
	} 
}

  /*********************************************************************************************************
  *	display futions
  *********************************************************************************************************/ 

// draw initial warehouse state
static void TaskStartDispInit()
{
	PC_DispStr(0, 0,  " ----01------02------03------04------05------06------07------08------09----  Transport Workspace  ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 1,  " ||      ll      ll      ll      ll      ll      ll      ll      ll      ll                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 2,  " l                                                                        l  18    Robots         ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 3,  " l                                                                        l        Running        ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 4,  " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l        Stopped        ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 5,  " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l        Sleeped        ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 6,  " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 7,  " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l       TP Orders       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 8,  " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l       LD Orders       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 9,  " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 10, " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l       Obstacles       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 11, " l                                                                        l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 12, " l                                                                        l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 13, " l                                                                        l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 14, " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 15, " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 16, " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 17, " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 18, " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 19, " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 20, " l    ■■      ■■      ■■      ■■      ■■      ■■      ■■    l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 21, " l                                                                        l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 22, " l                                                                        l                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 23, " ||      ll      ll      ll      ll      ll      ll      ll      ll      ll                       ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 24, " ----01------02------03------04------05------06------07------08------09----  Loading Workspace    ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
}

// erase robot's tajectory and infomation texts
static void TaskViewClear()
{
	INT8U i, j;
	char clear[3] = " ";

//	PC_DispStr(0, 1, " ||      ll      ll      ll      ll      ll      ll      ll      ll      ll", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 2, " l                                                                        l", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 3, " l                                                                        l        ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);

	for (i = 4; i < 11; i++) {
		PC_DispStr(2,  i, "    ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
		PC_DispStr(70, i, "    ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
		for (j = 1; j < 7; j++)
			PC_DispStr(j * 10, i, "      ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	}
		
/*	// clear texts
	PC_DispStr(79, 4, clear, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);	// 
	PC_DispStr(79, 5, clear, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);	// 
	PC_DispStr(79, 7, clear, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);	// 
	PC_DispStr(79, 8, clear, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);	//
	PC_DispStr(79, 10, clear, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);	// 
*/		
	PC_DispStr(0, 11, " l                                                                        l", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 12, " l                                                                        l", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 13, " l                                                                        l", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);

	for (i = 14; i < 21; i++) {
		PC_DispStr(2, i, "    ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
		PC_DispStr(70, i, "    ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
		for (j = 1; j < 7; j++)
			PC_DispStr(j * 10, i, "      ", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	}

	PC_DispStr(0, 21, " l                                                                        l", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	PC_DispStr(0, 22, " l                                                                        l", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
//	PC_DispStr(0, 23, " ||      ll      ll      ll      ll      ll      ll      ll      ll      ll", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);

}

// 로봇 + 목적지 그리기
void TaskViewDisp()
{
	INT8U i, ERR;
	INT16U robot_loc;

	// ****************** 뮤텍스 로봇 위치정보 보호 ******************
	OSMutexPend(MutexRobotSync, 0, &ERR);
	TaskViewClear();					// 로봇 흔적 지우기
	for (i = 0; i < N_ROBOT; i++) {		// 로봇 그리기 
		robot_loc = Pos2Loc(robots[i].pos);
		PC_DispStr((INT8U)(robot_loc >> 8), (INT8U)(robot_loc & 0xFF), "●", robots[i].color);
	}
	OSMutexPost(MutexRobotSync);
	// ****************** 뮤텍스 로봇 위치정보 보호 ******************
	
//	for(i = 0; i < )	// 장애물 그리기
	//  PC_DispStr((INT8U)(obstable_loc >> 8), (INT8U)(obstable_loc & 0xFF), "▲", STOP_COLOR);
}

void debug_print(INT8U row, const char* format, ...) 
{
	char str[80];
	char buf[17] = "0000000000000000";
	int length, tmp, idx = 0;

	va_list args;
	va_start(args, format);

	for (int i = 0; format[i]; i++) {
		if (format[i] == '%' && format[i+1] == 'd') {

			tmp = va_arg(args, int);

			for (int j = uint2str(buf, tmp) - 1; j >= 0; j--) {
				str[idx++] = buf[j];
				buf[j] = '0';
			}

			i += 1;
			continue;
		}
		else if (format[i] == '%' && format[i + 1] == 'b') {

			tmp = va_arg(args, int);

			for (int j = binary2str(buf, tmp, 16) - 1; j >= 0; j--) {
				str[idx++] = buf[j];
				buf[j] = '0';
			}

			i += 1;
			continue;
		}
		str[idx++] = format[i];
	}
	str[idx] = '\0';

	PC_DispStr(78, row, str, DISP_FGND_BLACK + DISP_BGND_WHITE);

	va_end(args);
}

int uint2str(char* strp, int n)
{
	int length = 0;

	for (int j = 0, i = 0; j < 10 && n != 0; j++, n /= 10) {
		i = n % 10;
		strp[length++] = i + '0';
	}

	return length;
}

int binary2str(char* strp, INT16U n, int length) 
{
	for (int i = 0; i < length; i++) {
		strp[i] = (n >> i & 1) + '0';
	}

	return length;
}