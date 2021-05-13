#include "includes.h"
// #define DEBUG_MODE
/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/

#define   TASK_STK_SIZE		512
#define       TASK_PRIO		16
#define		   SHELF2WS     1							// WareHouse to WorkSpace
#define		   WS2SHELF	    2							// WorkSpace to WareHouse
#define		  TRANSPORT     1							// WareHouse to WorkSpace
#define		      STORE	    2							// WorkSpace to WareHouse
#define    GLOBAL_ROUTE	    1
#define     LOCAL_ROUTE		2
#define   INITIAL_PLACE     0x01FF
#define        GETSHELF		0xFF

#define			N_ROBOT	    16
#define         N_ROUTE     70
#define		    N_COLOR	    6
#define         N_PLACE	    9
#define		  N_SHELVES	    196							// 2 * 7 * 7 * 2
#define   N_FIXED_BLOCK	    232							// N_SHELVES + N_PLACE * 2
#define        POS_LEFT	    2
#define         POS_TOP     1
#define       POS_WIDTH		36
#define      POS_HEIGHT     23
#define    N_PQ_ELEMENT	    600

#define ABS(x) (((x) < 0) ? -(x) : (x))

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


const int di[4] = { -1, 0, 1, 0 };
const int dj[4] = { 0, 1, 0, -1 };

/*
*********************************************************************************************************
*                                               VARIABLES
*********************************************************************************************************
*/

FILE* fp;
char logBuf[256];

OS_EVENT* MQOrder2Robot;					// 오더를 주면, 로봇이 선점해감
void* QOrderTbl[N_ROBOT + 1];

OS_EVENT* QRequestRoute;					// 로봇이 경로를 요청할 떄 필요
void* QBuff[N_ROBOT + 1];

OS_EVENT* SemReceiveRoute[N_ROBOT];			// 로봇이 경로를 요청하고, 받을 떄까지 대기하게 함
OS_EVENT* MutexRobotSync;					// 로봇의 위치정보를 가져오거나, 수정할 때 동기화를 위함
OS_EVENT* SemWorkSpace[2];

OS_STK TaskStk[4][TASK_STK_SIZE];
OS_STK TaskStk2[N_ROBOT][TASK_STK_SIZE];

typedef enum STATUS {
	IDLE,
	RUNNING,
	STOPED
}STATUS;

typedef struct Pos {
	INT8U i;
	INT8U j;
} Pos;

typedef struct ORDER_INFO {
	INT8U kind;
	INT8U workspace;
	INT8U shelf;
	INT8U color;
} ORDER_INFO;

typedef struct ROBOT_INFO {
	Pos pos;
	STATUS stat;
	INT8U color;
}ROBOT_INFO;

typedef struct REQUEST_INFO {
	Pos departure;
	Pos arrival;
	Pos* route;
	INT8U* steps;
	INT8U robot_no;
} REQUEST_INFO;

typedef struct PRIORITY_QUEUE {
	INT16U* val;
	Pos* loc;
	INT16U  size;
} PRIORITY_QUEUE;

typedef struct LIST {
	LIST* next;
	INT8U n;
} LIST;

OS_MEM* ListBuf;
INT8U ListPart[N_ROBOT + 1][sizeof(LIST)];

/*
typedef struct RouteRequestInfo {
	Pos start_pos;
	Pos arrival_pos;
	Pos* route;
	INT16U* max_step;
	INT8U kind;
	INT8U robot_no;
}RouteRequestInfo;
*/

Pos SHP[N_SHELVES];							// Shelves Point
Pos TPP[N_PLACE];							// Transport Point
Pos LDP[N_PLACE];							// Load Point
Pos PKP[N_ROBOT];							// Park Point
Pos BLP[N_FIXED_BLOCK+N_ROBOT];						// Blocked Point
INT8U isBlockedPOS[POS_HEIGHT][POS_WIDTH];

const Pos* shelves_ptr = SHP;
  const Pos* trans_ptr = TPP;
   const Pos* load_ptr = LDP;
   const Pos* park_ptr = PKP;
const Pos* blocked_ptr = BLP;

ROBOT_INFO robots[N_ROBOT];

LIST busyShelves;
// INT8U busyShelf[N_ROBOT];
// INT8U n_busy;

INT16U idle_tpp;
INT16U idle_ldp;

/*
INT32U idle_shelf[N_SHELVES / 32 + 1];
INT16U idle_park;
INT16U numOfOrder;
*/

// priority queue related
PRIORITY_QUEUE pq;
INT16U pq_val_buf[N_PQ_ELEMENT];
   Pos pq_loc_buf[N_PQ_ELEMENT];

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
// display related
static void TaskStartDispInit();
static void TaskViewClear();
void TaskViewDisp();

void workSpaceDisp(INT8U place, INT8U type, INT8U n);
void shelfDisp(INT8U shelf, INT8U n);

// initialize
void initialize_map();

// debug related 
int uint2str(char* strp, int n);
int binary2str(char* strp, INT16U n, int length);
void debug_print(INT8U row, const char* format, ...);
void RouteDisp(Pos* route, INT16U n);

// priority queue related
 __inline void pq_clear(PRIORITY_QUEUE* pq);
 __inline void pq_swap(void* a, void* b, int type);
		  void pq_push(PRIORITY_QUEUE* pq, INT16U v, Pos p);
	       Pos pq_pop(PRIORITY_QUEUE* pq);
__inline INT8U is_pq_empty(PRIORITY_QUEUE* pq);

// A* search
INT16U AStarSearch(Pos* route, const Pos start, const Pos end, const Pos* blocked_map, const INT16U n_blocked);

// other utils
__inline Pos Loc2Pos(INT16U loc);
__inline INT16U Pos2Loc(Pos pos);
__inline Pos frontOfShelf(Pos shelf)
__inline INT16U distance(Pos p1, Pos p2)
__inline INT8U isSamePos(const Pos p1, const Pos p2)

// get idle workspace, shelf
INT8U getIdleTurn(INT16U* code, INT8U size)
void returnTurn(INT16U* code, INT8U turn)
INT8U getIdleShelfNum()
void returnShelfNum(INT8U n)

void updatedBlockedMap();

/*
*********************************************************************************************************
*                                                MAIN
*********************************************************************************************************
*/

int main(void)
{
	INT8U i, ERR;

	if ((fp = fopen("log.txt", "w")) == NULL) exit(2);

	srand(time(0) + (OSTCBCur->OSTCBPrio * 237 >> 4));
	initialize_map();

	OSInit();
	
	// initialize task
	OSTaskCreate(TaskCentralControl, (void *)0, &TaskStk[0][TASK_STK_SIZE - 1], TASK_PRIO - 1);
	OSTaskCreate(TaskGetOrder,		 (void *)0, &TaskStk[1][TASK_STK_SIZE - 1], TASK_PRIO + N_ROBOT + 1);
	OSTaskCreate(TaskGetOrder,		 (void *)0, &TaskStk[2][TASK_STK_SIZE - 1], TASK_PRIO + N_ROBOT + 2);
	OSTaskCreate(TaskAssigntRoute,   (void *)0, &TaskStk[3][TASK_STK_SIZE - 1], TASK_PRIO);

//	for (i = 1; i < N_ROBOT+1; i++) {
//		OSTaskCreate(TaskRobotMove,  (void *)0, &TaskStk[i][TASK_STK_SIZE - 1], TASK_PRIO + i);
//	}
	OSTaskCreate(TaskRobotMove, (void *)0, &TaskStk2[0][TASK_STK_SIZE - 1], TASK_PRIO + 1);

	QRequestRoute = OSQCreate(QBuff, N_ROBOT + 1);
	MQOrder2Robot = OSQCreate(QOrderTbl, N_ROBOT + 1);
	MutexRobotSync = OSSemCreate(1);

	ListBuf = OSMemCreate(ListPart, N_ROBOT + 1, sizeof(LIST), &ERR);

	OSStart();

	return 0;
}

/*
*********************************************************************************************************
*                                          TASK CENTRAL CONTROL
*********************************************************************************************************
*/

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


		PC_GetDateTime(msg);
		PC_DispStr(78, 22, msg, DISP_FGND_YELLOW + DISP_BGND_BLUE);	// 

		TaskViewDisp();

		OSTimeDly(1);
	}
}

/*
*********************************************************************************************************
*                                          TASK ROBOT MOVE
*********************************************************************************************************
*/
void TaskRobotMove(void* pdata)
{ 
	const INT8U robot_no = OSTCBCur->OSTCBPrio % N_ROBOT;
	const INT8U d_value = 5;
	INT8U i, j, t_color, ERR, isBlocked = 0, stage = 0;
	INT8U stage = 0, step = 0, t_step, steps, t_steps;;
	INT16U* idle_place;

	ORDER_INFO order;
	REQUEST_INFO request;
	request.robot_no = robot_no;
	Pos g_dest[4];
	Pos* route;
	Pos g_route[N_ROUTE], l_route[N_ROUTE];

	ROBOT_INFO* robot = &robots[robot_no];
	robot->pos = park_ptr[robot_no];	// 로봇의 초기 위치 설정
	robot->stat = IDLE;

	SemReceiveRoute[robot_no] = OSSemCreate(1);

	while (1) {
#ifdef DEBUG_MODE
		fprintf(fp, "\n%d TaskRobotMove()\n", robot_no);
#endif

		order = *((OrderInfo *)OSQPend(MQOrder2Robot, 0, &ERR));			// waiting for order
		robot->stat = RUNNING;
		robot->color = order.color;

#ifdef DEBUG_MODE
		fprintf(fp, "%d TaskRobotMove() 오더 전달받음\n", robot_no);
#endif

		// set robot's destinations
		g_dest[0] = robot->pos;
		g_dest[3] = robot->pos;
		if (order.kind == TRANSPORT) {
			g_dest[1] = frontOfShelf(order.shelf);
			g_dest[2] = order.workspace;
			idle_place = &idle_tpp;
		}
		else if (order.kind == STORE) {
			g_dest[1] = order.workspace;
			g_dest[2] = frontOfShelf(order.shelf);
			idle_place = &idle_ldp
		} 

		for (stage = 0; stage < 3; stage++) {
			route = g_route;
			request.route     = route;
			request.departure = g_dest[stage];
			request.arrival   = g_dest[stage +1];
			request.steps     = &steps;

#ifdef DEBUG_MODE
			fprintf(fp, "\n%d TaskRobotMove() 전역경로 요청\n", robot_no);
			fprintf(fp, "%d TaskRobotMove() from (%d, %d) to (%d, %d)\n", request.departure.i, 
				request.departure.j, request.arrival.i, request.arrival.j);
#endif
			
			OSQPost(QRequestRoute, &request);								// request route to TaskAssignRoute
			OSSemPend(SemReceiveRoute[robot_no], 0, &ERR);					// receive robot's route	

#ifdef DEBUG_MODE
			fprintf(fp, "%d TaskRobotMove() 전역경로 할당 완료 steps: %d\n", robot_no, steps);
#endif

			// move along the route until arrival
			while (!isSamePos(request.arrival, robot->pos)) {

				// 전방에 다른 로봇이 있는지 탐색
				// ****************** 뮤텍스 로봇 위치정보 보호 ******************
				OSMutexPend(MutexRobotSync, 0, &ERR);
				for (i = 0, isBlocked = 0; i < N_ROBOT; i++) {
					if (i == robot_no) continue;
					if (isSamePos(robot->pos, robots[i].pos)) {
						isBlocked = 1;
						break;
					}
				}
				OSMutexPost(MutexRobotSync);
				// ****************** 뮤텍스 로봇 위치정보 보호 ******************

				// 로봇이 경로를 따라 진행 중 가로막혔을 때
				if (isBlocked) {	
#ifdef DEBUG_MODE
					fprintf(fp, "\n%d TaskRobotMove() 전역 경로 진행 중 장애물 식별\n", robot_no);
#endif
					t_color = robot->color;
					robot->stat = STOPED;
					robot->color = STOP_COLOR;

					OSTimeDly(1);

					route = l_route;
					request.route     = route;
					request.departure = robot->pos;
					if (steps > step + d_value) {
						request.arrival = g_route[step + d_value];
					}

					t_step  = step;
					step = 0;
					t_steps = steps;
					request.steps = &steps;

#ifdef DEBUG_MODE
					fprintf(fp, "\n%d TaskRobotMove() 지역경로 요청\n", robot_no);
					fprintf(fp, "%d TaskRobotMove() from (%d, %d) to (%d, %d)\n", request.departure.i,
						request.departure.j, request.arrival.i, request.arrival.j);
#endif
					// 경로 요청 완료. -> 메일박스로 받음
					OSQPost(QRequestRoute, &request);							// 목적지까지의 경로 요청
					OSSemPend(SemReceiveRoute[robot_no], 0, &ERR);				// 경로 받음

#ifdef DEBUG_MODE
					fprintf(fp, "%d TaskRobotMove() 지역경로 할당 완료 steps: %d\n", robot_no, steps);
#endif

					robot->stat = RUNNING;
					robot->color = t_color;

					OSTimeDly(1);			// 우선순위가 큰 로봇이 계속해서 선점하는 것을 방지

					continue;
				}	// isBlocked

				// 로봇 경로를 따라 한 칸 이동
				// ****************** 뮤텍스 로봇 위치정보 보호 ******************
				OSMutexPend(MutexRobotSync, 0, &ERR);

				robot->pos.i = (route + step)->i;
				robot->pos.j = (route + step)->j;

				OSMutexPost(MutexRobotSync);
				// ****************** 뮤텍스 로봇 위치정보 보호 ******************
				step += 1;
#ifdef DEBUG_MODE
				fprintf(fp, "%d TaskRobotMove() 로봇 이동 (%d, %d)\n", robot_no, robot->pos.i, robot->pos.j);
#endif

				// 다음 경우가 성립하는 상황은 로컬경로를 진행중일 때 만
				if ((step == steps) && (route == l_route)) {
					step  = t_step + d_value;
					steps = t_steps;
					route = g_route;
#ifdef DEBUG_MODE
					fprintf(fp, "%d TaskRobotMove() 지역경로 도착, 전역경로 합류\n", robot_no);
#endif
				}
#ifdef DEBUG_MODE
				else if (step > steps) {
					fprintf(fp, "TaskRobotMove() 경로 문제 발생");
					exit(1);
				}
				fprintf(fp, "robot의 %번째 위치: (%d, %d)", step, robot->pos.i, robot->pos.j);
#endif

				OSTimeDly(1);
			}
		}

#ifdef DEBUG_MODE
		fprintf(fp, "%d TaskRobotMove() 도착\n", robot_no, steps);
#endif

		// 로봇의 작업이 끝난 후 로봇의 상태 및 장업장의 상태를 유휴상태로 되돌리기.

		robot->stat = IDLE;
		robot->color = IDLE_COLOR;

		returnTurn(idle_place, order.workspace);
		returnShelfNum(order.shelf);
		workSpaceDisp(order.workspace, order.kind, 2);						// recover workspace display
		shelfDisp(order.shelf, IDLE_COLOR);									// recover color shelf

		OSSemPost(SemWorkSpace[kind]);										// idle workplace num added 1

#ifdef DEBUG_MODE
		fprintf(fp, "%d TaskRobotMove() SemWorkSpace[%d] 반환\n", robot_no, kind);
#endif
		
		OSTimeDly(1);
	}
}

/*
*********************************************************************************************************
*                                          TASK ASSIGN ROUTE
*********************************************************************************************************
*/
void TaskAssigntRoute(void* pdata) 
{
	INT8U ERR;
	REQUEST_INFO request;

	while (1) {
#ifdef DEBUG_MODE
		fprintf(fp, "TaskAssigntRoute() \n");
#endif

		request = *((REQUEST_INFO *)OSQPend(QRequestRoute, 0, &ERR));

#ifdef DEBUG_MODE
		fprintf(fp, "TaskAssigntRoute() 경로요청 수신\n");
#endif

		updatedBlockedMap();									// update blocked pos
		// search route
		request->steps = AStarSearch(request->route, request->departure, request->arrival,
			blocked_ptr, N_FIXED_BLOCK + N_ROBOT);
		
		OSSemPost(SemReceiveRoute[request.robot_no]);			// complete assign route

#ifdef DEBUG_MODE
		fprintf(fp, "TaskAssigntRoute() 로봇 %d 경로할당 완료\n", robot_no);
#endif

		OSTimeDly(1);
	}
}

/*
*********************************************************************************************************
*                                            TASK GET ORDER
*********************************************************************************************************
*/
void TaskGetOrder(void* pdata) 
{
	INT8U i, ERR, kind, shelf, place;
	INT16U* idle_place;
	ORDER_INFO order;
	
	kind = OSTCBCur->OSTCBPrio % 2 + 1;
	order.kind = kind;

	if (kind == TRANSPORT) {
		idle_place = &idle_tpp;
	}
	else if (kind == STORE) {
		idle_place = &idle_ldp
	}
	*idle_place = INITIAL_PLACE;
	
	SemWorkSpace[kind] = OSSemCreate(N_PLACE);				// 뮤텍스로 바꿔

	while (1) {
		OSSemPend(SemWorkSpace[kind], 0, &ERR);			// check that all places are busy
		place = getIdleTurn(idle_place, N_PLACE);
		shelf = getIdleShelfNum();

		order.workspace = place;
		order.shelf     = shelf;
		order.color     = Colors[(INT8U)(rand() % N_COLOR)];

		workSpaceDisp(place, kind, 1);								// display '*' workspace
		shelfDisp(shelf, color);									// display color shelf

#ifdef DEBUG_MODE
		fprintf(fp, "TaskGetOrder kind %d workspace %d shelf %d\n", kind, place, shelf);
		fprintf(fp, "idle_place: %b", *idle_place);
#endif

//		OSMboxPost(MOrder2Robot[robot], &order);	// send order to the robot task by the robot's mailbox
		OSQPost(MQOrder2Robot, &order);
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

// 로봇의 위치 갱신하며 그리기
void TaskRobotsDisp()
{
	INT8U i;
	INT16U loc;

	TaskViewClear();
	for (i = 0; i < N_ROBOT; i++) {
		loc = Pos2Loc(robots[i].pos);
		PC_DispStr((loc >> 8), (loc & 0xFF), "●", robots[i].color);
	}
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

	/*	// clear texts
	PC_DispStr(79, 4, clear, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);	//
	PC_DispStr(79, 5, clear, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);	//
	PC_DispStr(79, 7, clear, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);	//
	PC_DispStr(79, 8, clear, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);	//
	PC_DispStr(79, 10, clear, DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);	//
*/
}

// n == 1 작업장 표시하기
// n == 2 작업장 원래대로 돌리기
void workSpaceDisp(INT8U place, INT8U type, INT8U n) 
{
	if (n == 1) {
		if (type == TRANSPORT) {
			PC_DispStr(2 + 8 * place, 1, "*", order.color);					// update display
			PC_DispStr(2 + 8 * (place + 1) - 1, 1, "*", order.color);		// update display
		}
		else if (type == STORE) {
			PC_DispStr(2 + 8 * place, 23, "*", order.color);				// update display
			PC_DispStr(2 + 8 * (place + 1) - 1, 23, "*", order.color);		// update display
		}
	}
	else if (n == 2) {
		if (type == TRANSPORT) {
			PC_DispStr(2 + 8 * place, 1, "l", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);					// update display
			PC_DispStr(2 + 8 * (place + 1) - 1, 1, "l", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);		// update display
		}
		else if (type == STORE) {
			PC_DispStr(2 + 8 * place, 23, "l", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);				// update display
			PC_DispStr(2 + 8 * (place + 1) - 1, 23, "l", DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY);		// update display
		}
	}
}

// 작업선반 color로 표시하기
// color == DISP_FGND_BLACK + DISP_BGND_LIGHT_GRAY 작업선반 원래대로 돌리기
void shelfDisp(INT8U shelf, INT8U color)
{
	INT16U loc = Pos2Loc(shelves_ptr[shelf]);

	PC_DispStr((loc >> 8), (loc & 0xFF), "■", color);
}

/*
*********************************************************************************************************
*                                          PRIORITY QUEUE (오름차순)
*********************************************************************************************************
*/
__inline void pq_clear(PRIORITY_QUEUE* pq)
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

void pq_push(PRIORITY_QUEUE* pq, INT16U v, Pos p)
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
Pos pq_pop(PRIORITY_QUEUE* pq)
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

__inline INT8U is_pq_empty(PRIORITY_QUEUE* pq)
{
	return (pq->size == 0);
}

/*
*********************************************************************************************************
*                                             A* SEARCH
*********************************************************************************************************
*/
// return route's num
// weight 설정 1순위: 시작점에서 크게 벗어나지 않게, 2순위: 도착점을 향하도록
INT16U AStarSearch(Pos* route, const Pos start, const Pos end, const Pos* blocked, const INT16U n_blocked) 
{
	static PRIORITY_QUEUE* ppq;				// this PRIORITY_QUEUE is only for A* search
	if (ppq != (void*)0) {
		ppq = &pq;
		ppq->val = pq_val_buf;
		ppq->val = pq_loc_buf;
		ppq->size = 0;
	}
	
	INT8U n = 0, test_val;
	Pos cur, next, temp;
	Pos came_from[POS_HEIGHT][POS_WIDTH];
	memset(came_from, 0, sizeof(Pos)*POS_HEIGHT*POS_WIDTH);

	INT8U weight[POS_HEIGHT][POS_WIDTH];
	memset(weight, 0xFF, sizeof(INT8U)*POS_HEIGHT*POS_WIDTH);
	for (INT16U i = 0; i < n_blocked; i++) {
		temp = blocked[i];
		weight[temp.i][temp.j] = 0;
	}

	// find route from arrival to start 
	pq_push(ppq, 0, end);
	weight[end.i][end.j] = 1;
	while (!is_pq_empty(ppq)) {
		cur = pq_pop(ppq);
		if (isSamePos(cur, start))			// arrive to destination
			break;

		for (INT8U k = 0; k < 4; k++) {		// search for 4 direction
			next.i = cur.i + di[k];
			next.j = cur.j + dj[k];

			if (!(next.i >= 0 && next.i < POS_HEIGHT && next.j >= 0 && next.j < POS_WIDTH))
				continue;

			if (weight[next.i][next.j] > weight[cur.i][cur.j] + 1) {
				if (weight[next.i][next.j] == 0xFF) {
					pq_push(ppq, 3 * distance(next, end) + 2 * distance(next, start), next);
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
	if (is_pq_empty(ppq)) {
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

	pq_clear(ppq);

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

// p[0].i = 3;  p[0].j = 2 + 5 * i; p[1].i = 3;  p[1].j = 3 + 5 * i;
// p[2].i = 13; p[2].j = 2 + 5 * i; p[3].i = 13; p[3].j = 3 + 5 * i;
__inline Pos frontOfShelf(Pos shelf) 
{
	if (shelf.j % 5 == 2) {
		shelf.j -= 1;
	}
	else if (shelf.j % 5 == 3) {
		shelf.j += 1;
	}
	return shelf;
}

__inline INT16U distance(Pos p1, Pos p2)
{
	return (ABS(p1.i - p2.i) + ABS(p1.j - p2.j));
}

__inline INT8U isSamePos(const Pos p1, const Pos p2) 
{
	return (p1.i == p2.i) && (p1.j == p2.j);
}

INT8U getIdleTurn(INT16U* code, INT8U size) 
{
	INT8U n = (INT8U)(rand() % size);

	while (*code & (1 << n)) {
		n = (n + 1) % size;
	}
	*code = *code - (1 << n);

	return n;
}

void returnTurn(INT16U* code, INT8U turn) 
{
	*code = *code | (1 << turn);
}

INT8U getIdleShelfNum() 
{
	INT8U n = rand() % N_SHELVES, ERR;
	LIST* head = &busyShelves;
	LIST* temp = OSMemGet(ListBuf, &ERR);
	
	while (!head) {
		if (n == head->n) {
			n = rand() % N_SHELVES;
			head = &busyShelves;
		}
		head = head->next;
	}

	head = &busyShelves;
	temp->n = n;
	temp->next = head->next;
	head->next = temp;

	return n;
}

void returnShelfNum(INT8U n) 
{
	LIST* head = &busyShelves;
	LIST* previous = head;
	
	while (!head) {
		if (n == head->n) {
			previous->next = head->next;
			OSMemPut(ListBuf, head);
			break;
		}
		previous = head;
		head = head->next;
	}
}

void updatedBlockedMap() 
{
	for (INT8U n = 0; n < N_ROBOT; n++) 
		blocked_ptr[N_FIXED_BLOCK + n] = robots[n].pos;
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
	memset(isBlockedPos, 0, sizeof(INT8U)*POS_HEIGHT*POS_WIDTH);

	INT8U n = 0, m = 0;
	Pos p[4], temp;

	// set blocked point, park point
	p[0].i = 0;  p[0].j = 0; p[1].i = 0;  p[1].j = 3;
	p[2].i = 22; p[2].j = 0; p[3].i = 22; p[3].j = 3;
	for (INT8U i = 0; i < 9; i++) {
		for (INT8U j = 0; j < 4; j++) {
//			BLP[n++] = p[j];
			isBlockedPos[p[j].i][p[j].j] = 1;

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
				SHP[n++] = p[k];
				isBlockedPos[p[k].i][p[k].j] = 1;

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
			if(isBlockedPos[i][j] == 1)
				fprintf(fp, "# ");
		}
		fprintf(fp, "\n");
	}
	fprintf(fp, "\n\n");

	INT8U copy[POS_HEIGHT][POS_WIDTH];
	memcpy(copy, isBlockedPos, sizeof(INT8U)*POS_HEIGHT*POS_WIDTH);

	for (int i = 0; i < N_PLACE; i++) {
		temp = LDP[i];
		isBlockedPos[temp.i][temp.j] = 2;
		temp = TPP[i];
		isBlockedPos[temp.i][temp.j] = 3;
	}

	for (int i = 0; i < N_ROBOT; i++) {
		temp = PKP[i];
		isBlockedPos[temp.i][temp.j] = 4;
	}

	for (int i = 0; i < N_SHELVES; i++) {
		temp = SHP[i];
		isBlockedPos[temp.i][temp.j] = 5;
	}

	for (int i = 0; i < POS_HEIGHT; i++) {
		for (int j = 0; j < POS_WIDTH; j++) {
			switch (isBlockedPos[i][j]) {
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

	memcpy(isBlockedPos, copy, sizeof(INT8U)*POS_HEIGHT*POS_WIDTH);

	// exit(1);
#endif
}