#include "includes.h"
// #define DEBUG_MODE
/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/

#define SHELF2WS		  1							// WareHouse to WorkSpace
#define WS2SHELF		  2							// WorkSpace to WareHouse
#define N_ROBOT			 16
#define N_COLOR			  6
#define N_SHELVES	    196							// 2 * 7 * 7 * 2
#define N_PLACE		      9
#define TASK_STK_SIZE	512
#define TASK_PRIO        16
#define INITIAL_PLACE     0b0000000111111111

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

#define ABS(x) (((x) < 0) ? -(x) : (x))
#define     POS_WIDTH	 36
#define    POS_HEIGHT    23
#define  N_PQ_ELEMENT	 600
#define N_FIXED_BLOCK	 232

const int di[4] = { -1, 0, 1, 0 };
const int dj[4] = { 0, 1, 0, -1 };

/*
*********************************************************************************************************
*                                               VARIABLES
*********************************************************************************************************
*/

FILE* fp;
char logBuf[256];

OS_EVENT* MOrder2Robot[N_ROBOT];			// 오더를 받아, 로봇에게 전달할 떄 사용되는 메일박스
OS_EVENT* QRequestRoute;					// 로봇이 경로를 요청할 떄 필요
OS_EVENT* SemReceiveRoute[N_ROBOT];			// 로봇이 경로를 요청하고, 받을 떄까지 대기하게 함
OS_EVENT* MutexRobotSync;					// 로봇의 위치정보를 가져오거나, 수정할 때 동기화를 위함
OS_EVENT* SemWorkSpace[2];
OS_STK TaskStk[4][TASK_STK_SIZE];
OS_STK TaskStk2[N_ROBOT][TASK_STK_SIZE];

Pos SHP[N_SHELVES];							// Shelves Point
Pos TPP[N_PLACE];							// Transport Point
Pos LDP[N_PLACE];							// Load Point
Pos PKP[N_ROBOT];							// Park Point
Pos BLP[N_FIXED_BLOCK];						// Blocked Point
INT8U isBlocked[POS_HEIGHT][POS_WIDTH];

const Pos* shelves_p = SHP;
const Pos* trans_p = TPP;
const Pos* load_p = LDP;
const Pos* park_p = PKP;

RobotInfo robots[N_ROBOT];

INT32U idle_shelf[N_SHELF / 32 + 1];
INT16U idle_tpp;
INT16U idle_ldp;
INT16U idle_park;
INT16U numOfOrder;

typedef struct {
	INT8U i;
	INT8U j;
} Pos;

// priority queue related
INT16U pq_val_buf[N_PQ_ELEMENT];
   Pos pq_loc_buf[N_PQ_ELEMENT];

typedef struct {
	INT16U* val;
	Pos* loc;
	INT16U  size;
} priority_queue;

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void TaskCentralControl(void* pdata);	// 중앙처리
void TaskRobotMove(void* pdata);		// 24개, 이 태스크는 길이 필요할 시 아래 assignRoute에 요청할 수 있음 
void TaskGetOrder(void* pdata);			// 오더를 받는 함수, 우선순위 가장 낮음
void TaskAssigntRoute(void* pdata);		// 태스크가 자원을 많이 사용함 -> 한 주기에 사용할 수 있는 제한을 건다.
										// 메세지에 따라 해당 로봇에게 주는데, 1 전역 2 지역
static void TaskStartDispInit();
static void TaskViewClear();
void TaskViewDisp();

int uint2str(char* strp, int n);
int binary2str(char* strp, INT16U n, int length);
void debug_print(INT8U row, const char* format, ...);
void RouteDisp(Pos* route, INT16U n);

// priority queue related
 __inline void pq_clear(priority_queue* pq);
 __inline void pq_swap(void* a, void* b, int type);
		  void pq_push(priority_queue* pq, INT16U v, Pos p);
	       Pos pq_pop(priority_queue* pq);
__inline INT8U is_pq_empty(priority_queue* pq);

// other util function
__inline struct Pos Loc2Pos(INT16U loc);
    __inline INT16U Pos2Loc(Pos pos);
	__inline INT16U frontOfShelf(INT16U shelf_no, INT16U shelf_loc);
	__inline INT16U originalShelf(INT16U shelf_loc);
	__inline INT16U distance(Pos p1, Pos p2);
	 __inline INT8U isSamePos(const Pos p1, const Pos p2);

/*
*********************************************************************************************************
*                                                MAIN
*********************************************************************************************************
*/

int main(void)
{
	INT8U i;
	void* QBuff[N_ROBOT+1];

	if ((fp = fopen("log.txt", "w")) == NULL) exit(2);

	initialize_map();

	OSInit();

	// initialize idle codes
	idle_tpp    = 0b111111111;
	idle_ldp    = 0b111111111;
	idle_park   = 0;	// 모든 로봇이 주차된 상태
	for (i = 0; i < N_SHELVES / 32 + 1; i++) 
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

	// *******************************   test   *******************************
//	const Pos* ShelfPos_ptr = setShelfPos(ShelfPos);
//	const Pos* TPPos_ptr = setTPPos(TPPos, N_PLACE);
//	const Pos* LDPos_ptr = setLDPos(LDPos, N_PLACE);
//	setblockedPos(blockedPos, ShelfPos, N_SHELF);
//	const INT8U(*blocked_ptr)[WIDTH] = blockedPos;

	INT16U step;
	Pos route[300];
	Pos start = ShelfPos_ptr[5];
	Pos arrival = LDPos_ptr[5];
	// *******************************   test   *******************************

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

		AStarSearch(route, blocked_ptr, &step, start, arrival);
		RouteDisp(route, step);

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
/*
	while (1) {
		request = *((RouteRequestInfo*)OSQPend(QRequestRoute, 0, &ERR));	// 로봇으로 부터 경로할당 요청 수신

		AStarSearch(request.route, blocked_ptr, request.max_step, request.start_pos, request.arrival_pos);
		OSSemPost(SemReceiveRoute[request.robot_no]);	// 할당 완료

		if (request.kind == GLOBAL_ROUTE) OSTimeDly(1);
		else if (request.kind == LOCAL_ROUTE) {			// 지역 경로 요청
			if (--cnt) {
				cnt = 4;;
				OSTimeDly(1);
			}
		}
	}
*/
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

/*
*********************************************************************************************************
*                                           Display Related Task
*********************************************************************************************************
*/
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

/*	// ****************** 뮤텍스 로봇 위치정보 보호 ******************
	OSMutexPend(MutexRobotSync, 0, &ERR);
	TaskViewClear();					// 로봇 흔적 지우기
	for (i = 0; i < N_ROBOT; i++) {		// 로봇 그리기 
		robot_loc = Pos2Loc(robots[i].pos);
		PC_DispStr((INT8U)(robot_loc >> 8), (INT8U)(robot_loc & 0xFF), "●", robots[i].color);
	}
	OSMutexPost(MutexRobotSync);
*/	// ****************** 뮤텍스 로봇 위치정보 보호 ******************
	
//	for(i = 0; i < )	// 장애물 그리기
	//  PC_DispStr((INT8U)(obstable_loc >> 8), (INT8U)(obstable_loc & 0xFF), "▲", STOP_COLOR);
}

/*
*********************************************************************************************************
*                                          Priority Queue (오름차순)
*********************************************************************************************************
*/
__inline void pq_clear(priority_queue* pq)
{
	pq->size = 0;
	return;
}

__inline void pq_swap(void* a, void* b, int type)
{
	if (type == 1) {
		INT16U temp;
		temp = *(INT16U *)a;
		*(INT16U *)a = *(INT16U *)b;
		*(INT16U *)b = temp;
	}
	else if (type == 2) {
		Pos temp;
		temp = *(Pos *)a;
		*(Pos *)a = *(Pos *)b;
		*(Pos *)b = temp;
	}
}

void pq_push(priority_queue* pq, INT16U v, Pos p)
{
	INT16U idx = ++pq->size;
	INT16U next = idx / 2;
	INT16U* v_arr = pq->val;
	Pos* p_arr = pq->loc;

	pq->val[idx] = v;
	pq->loc[idx] = p;

	while ((v_arr[idx] < v_arr[next]) && (next > 0))
	{
		pq_swap(&v_arr[idx], &v_arr[next], 1);
		pq_swap(&p_arr[idx], &p_arr[next], 2);

		idx = next;
		next /= 2;
	}
}
Pos pq_pop(priority_queue* pq)
{
	INT16U idx = 1;
	INT16U next = idx * 2;
	INT16U* v_arr = pq->val;
	Pos* p_arr = pq->loc;
	Pos result;

	result = p_arr[idx];
	p_arr[idx] = p_arr[pq->size];
	v_arr[idx] = v_arr[pq->size];
	pq->size -= 1;

	while (next <= pq->size)						// 자식 노드가 하나 이상 남아있음. 
	{
		if (next != pq->size)						// 오른쪽 자식이 존재하고
			if (v_arr[next] > v_arr[next + 1])		// 오른쪽 자식이 더 작을 때
				next += 1;

		if (v_arr[idx] > v_arr[next]) {
			pq_swap(&v_arr[idx], &v_arr[next], 1);
			pq_swap(&p_arr[idx], &p_arr[next], 2);
		}
		else break;

		idx = next;
		next *= 2;									// 현재 노드의 왼쪽 자식 노드의 인덱스
	}

	return result;
}

__inline INT8U is_pq_empty(priority_queue* pq)
{
	return (pq->size == 0);
}

/*
*********************************************************************************************************
*                                             AStarSearch
*********************************************************************************************************
*/
// return route's num
// weight 설정 1순위: 시작점에서 크게 벗어나지 않게, 2순위: 도착점을 향하도록
INT16U AStarSearch(Pos* route, const Pos start, const Pos end, priority_queue* pq, const Pos* blocked_map
	, const INT16U n_blocked) 
{
	INT8U n = 0, test_val;
	Pos cur, next, temp;
	Pos came_from[POS_HEIGHT][POS_WIDTH];
	memset(came_from, 0, sizeof(Pos)*POS_HEIGHT*POS_WIDTH);

	INT8U weight[POS_HEIGHT][POS_WIDTH];
	memset(weight, 0xFF, sizeof(INT8U)*POS_HEIGHT*POS_WIDTH);
	for (INT16U i = 0; i < n_blocked; i++) {
		temp = blocked_map[i];
		weight[temp.i][temp.j] = 0;
	}

	// find route from arrival to start 
	pq_push(pq, 0, end);
	weight[end.i][end.j] = 1;
	while (!is_pq_empty(pq)) {
		cur = pq_pop(pq);
		if (isSamePos(cur, start))			// arrive to destination
			break;

		for (INT8U k = 0; k < 4; k++) {		// search for 4 direction
			next.i = cur.i + di[k];
			next.j = cur.j + dj[k];

			if (!(next.i >= 0 && next.i < POS_HEIGHT && next.j >= 0 && next.j < POS_WIDTH))
				continue;

			if (weight[next.i][next.j] > weight[cur.i][cur.j] + 1) {
				if (weight[next.i][next.j] == 0xFF) {
					pq_push(pq, 3 * distance(next, end) + 2 * distance(next, start), next);
				}
				weight[next.i][next.j] = weight[cur.i][cur.j] + 1;
				came_from[next.i][next.j] = cur;
			}

#ifdef DEBUG_MODE
			// fprintf(fp, " (%02d, %02d) ", next.i, next.j);
#endif 

		}

#ifdef DEBUG_MODE
		// fprintf(fp, " current node: (%d, %d) %d\n", cur.i, cur.j, weight[cur.i][cur.j]);
#endif 
	}

#ifdef DEBUG_MODE
	if (is_pq_empty(pq)) {
		fprintf(fp, "\nAStarSearch() 경로 탐색 단계 에러: 우선순위 큐에 원소가 없음\n");
		fprintf(fp, "시작점: (%d, %d) , 도착점: (%d, %d)\n", start.i, start.j, end.i, end.j);

		for (int i = 0; i < POS_HEIGHT; i++) {
			for (int j = 0; j < POS_WIDTH; j++) {
				if (weight[i][j] == 0) {
					fprintf(fp, "# ");
				}
				else if (weight[i][j] != 0xFF) {
					fprintf(fp, "  ");
				}
				else {
					fprintf(fp, "* ");
				}
			}
			fprintf(fp, "\n");
		}
		fprintf(fp, "\n");
		exit(1);
	}
#endif 

	pq_clear(pq);

	temp = start;
	while (!isSamePos(temp, end)) {
		route[n++] = temp;
		temp = came_from[temp.i][temp.j];

#ifdef DEBUG_MODE
		if (n > POS_HEIGHT + POS_WIDTH) {
			fprintf(fp, "AStarSearch() 경로의 수가 정상적이 않음");
			return 0;
		}
#endif
	}
	route[n++] = temp;	// 마지막

	return n;
}

/*
*********************************************************************************************************
*                                          Other Util Function
*********************************************************************************************************
*/


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

__inline INT16U distance(Pos p1, Pos p2)
{
	return (ABS(p1.i - p2.i) + ABS(p1.j - p2.j));
}

__inline INT8U isSamePos(const Pos p1, const Pos p2) {
	return (p1.i == p2.i) && (p1.j == p2.j);
}


/*
*********************************************************************************************************
*                                          Debug Related Function
*********************************************************************************************************
*/

void RouteDisp(Pos* route, INT16U n)
{
	INT16U loc;
	for (INT16U i = 0; i < n; i++) {
		loc = Pos2Loc(route[i]);
		PC_DispStr((INT8U)(loc >> 8), (INT8U)(loc & 0xFF), "◇", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);
	}
	return;
}

void debug_print(INT8U row, const char* format, ...)
{
	char str[80];
	char buf[17] = "0000000000000000";
	int length, tmp, idx = 0;

	va_list args;
	va_start(args, format);

	for (int i = 0; format[i]; i++) {
		if (format[i] == '%' && format[i + 1] == 'd') {

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

/*
*********************************************************************************************************
*                                           INITIALIZE MAP
*********************************************************************************************************
*/
// set up map (SHP, TPP, LDP, PKP, BLP, isBlocked)
void initialize_map()
{
	memset(isBlocked, 0, sizeof(INT8U)*POS_HEIGHT*POS_WIDTH);

	INT8U n = 0, m = 0;
	Pos p[4], temp;

	// set blocked point, park point
	p[0].i = 0;  p[0].j = 0; p[1].i = 0;  p[1].j = 3;
	p[2].i = 22; p[2].j = 0; p[3].i = 22; p[3].j = 3;
	for (INT8U i = 0; i < 9; i++) {
		for (INT8U j = 0; j < 4; j++) {
			BLP[n++] = p[j];
			isBlocked[p[j].i][p[j].j] = 1;

			temp = p[j];
			if (j == 1 && i != 8) {
				temp.i += 1;
				PKP[m++] = temp;
			}
			else if (j == 2 && i != 0) {
				temp.i -= 1;
				PKP[m++] = temp;
			}

			p[j].j += 4;
		}
	}
	m = 0;

	// set blocked point, selves point
	for (INT8U i = 0; i < 7; i++) {
		p[0].i = 3;  p[0].j = 2 + 5 * i; p[1].i = 3;  p[1].j = 3 + 5 * i;
		p[2].i = 13; p[2].j = 2 + 5 * i; p[3].i = 13; p[3].j = 3 + 5 * i;
		for (INT8U j = 0; j < 7; j++) {
			for (INT8U k = 0; k < 4; k++) {
				BLP[n++] = p[k];
				isBlocked[p[k].i][p[k].j] = 1;

				temp = p[k];
				if (k % 2 == 0) {
					temp.j -= 1;
					SHP[m++] = temp;
				}
				else {
					temp.j += 1;
					SHP[m++] = temp;
				}

				p[k].i += 1;
			}
		}
	}
	n = m = 0;

	// set transport point, load point
	p[0].i = 0; p[0].j = 2; p[1].i = 22; p[1].j = 1;
	for (int i = 0; i < N_PLACE; i++) {
		TPP[i] = p[0];
		LDP[i] = p[1];
		p[0].j += 4;
		p[1].j += 4;
	}

#ifdef DEBUG_MODE
	// ***** test code *****
	fprintf(fp, "\n\n");
	for (int i = 0; i < POS_HEIGHT; i++) {
		for (int j = 0; j < POS_WIDTH; j++) {
			if(isBlocked[i][j] == 1)
				fprintf(fp, "# ");
		}
		fprintf(fp, "\n");
	}
	fprintf(fp, "\n\n");

	INT8U copy[POS_HEIGHT][POS_WIDTH];
	memcpy(copy, isBlocked, sizeof(INT8U)*POS_HEIGHT*POS_WIDTH);

	for (int i = 0; i < N_PLACE; i++) {
		temp = LDP[i];
		isBlocked[temp.i][temp.j] = 2;
		temp = TPP[i];
		isBlocked[temp.i][temp.j] = 3;
	}

	for (int i = 0; i < N_ROBOT; i++) {
		temp = PKP[i];
		isBlocked[temp.i][temp.j] = 4;
	}

	for (int i = 0; i < N_SHELVES; i++) {
		temp = SHP[i];
		isBlocked[temp.i][temp.j] = 5;
	}

	for (int i = 0; i < POS_HEIGHT; i++) {
		for (int j = 0; j < POS_WIDTH; j++) {
			switch (isBlocked[i][j]) {
			case 0:
				fprintf(fp, "  ");
				break;
			case 1:
				fprintf(fp, "B ");
				break;
			case 2:
				fprintf(fp, "L ");
				break;
			case 3:
				fprintf(fp, "T ");
				break;
			case 4:
				fprintf(fp, "P ");
				break;
			case 5:
				fprintf(fp, "S ");
				break;
			}
		}
		fprintf(fp, "\n");
	}
	fprintf(fp, "\n\n");

	memcpy(isBlocked, copy, sizeof(INT8U)*POS_HEIGHT*POS_WIDTH);

	// exit(1);
#endif
}