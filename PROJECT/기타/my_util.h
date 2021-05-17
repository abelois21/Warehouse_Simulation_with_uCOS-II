/*
*********************************************************************************************************
*                                                uC/OS-II
*                                          The Real-Time Kernel
*					
*********************************************************************************************************
*/


#ifndef _MY_UTIL_H_
#define _MY_UTIL_H_

#include "includes.h"

#define SHELF2WS		 1	// WareHouse to WorkSpace
#define WS2SHELF		 2	// WorkSpace to WareHouse

#define GLOBAL_ROUTE	 1
#define LOCAL_ROUTE		 2
#define MAX_QUEUE		 500

#define POS_LEFT	     2
#define POS_TOP          1
#define POS_RIGHT	     78
#define POS_BOTTOM	     23
#define WIDTH			 39
#define HEIGHT			 23

#define    POS_WIDTH	36
#define   POS_HEIGHT    23
#define N_PQ_ELEMENT	600
#define    N_BLOCKED	214

#define INITIAL_ELEMENT		 400

typedef enum STATUS {
	IDLE,
	RUNNING,
	STOPED
}STATUS;

typedef struct Pos {
	INT8U i;
	INT8U j;
}Pos;

typedef struct OrderInfo {
	INT16U start_loc;	
	INT16U arrival_loc;	
	INT8U color;
	INT8U kind;
} OrderInfo;

typedef struct RobotInfo {
	Pos pos;
	STATUS stat;
	INT8U color;
}RobotInfo;

typedef struct RouteRequestInfo {
	Pos start_pos;
	Pos arrival_pos;
	Pos* route;
	INT16U* max_step;
	INT8U kind;
	INT8U robot_no;
}RouteRequestInfo;

// 우선순위 큐의 index 1부터 시작, index 0은 무효
typedef struct {
	Pos pos[1 + MAX_QUEUE];
	INT16U weight[1 + MAX_QUEUE];
	INT16U cnt;							
} PQ;

Pos* setShelfPos(Pos* ShelfPos);
Pos* setParkingPos(Pos* ParkingPos, INT16U num);
Pos* setTPPos(Pos* TPP, INT16U num);
Pos* setLDPos(Pos* LDP, INT16U num);
void setblockedPos(INT8U(*blockedPos)[WIDTH], const Pos* ShelfPos, INT16U num);

__inline INT16U distance(Pos p1, Pos p2);
__inline Pos ij2Pos(INT8U i, INT8U j);

INT8U getIdleStuff(INT32U* idle_code, INT8U idle_bound, INT8U idx);
INT16U getIdleShelf(INT32U* idle_shelf, INT8U idle_bound, INT8U idx);


void AStarSearch(Pos* route, const INT8U(*isBlocked)[WIDTH], INT16U* n_route, const Pos start, const Pos arrive);

INT8U isBlockedPos(Pos p);

typedef struct {
	INT16U* val;
	Pos* loc;
	INT16U  size;
} priority_queue;

__inline void pq_clear(priority_queue* pq);
__inline void pq_swap(void* a, void* b, int type);
void pq_push(priority_queue* pq, INT16U v, Pos p);
Pos pq_pop(priority_queue* pq);
__inline INT8U is_pq_empty(priority_queue* pq);


#endif // _MY_UTIL_H_

