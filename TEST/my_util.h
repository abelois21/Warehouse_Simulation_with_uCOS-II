/*
*********************************************************************************************************
*                                                uC/OS-II
*                                          The Real-Time Kernel
*
*					WIN32 PORT & LINUX PORT
*                          (c) Copyright 2004-... Werner.Zimmermann@fht-esslingen.de
*                                           All Rights Reserved
*
*               This file should contain application specific definitions, required since V2.8x
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

#define ABS(x)			 ((x > 0) ? x : -x)

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

void pushPQ(PQ* root, Pos newone, INT16U weight);
Pos popPQ(PQ* root);
void swap(PQ* root, INT16U idx1, INT16U idx2);

void AStarSearch(Pos* route, const INT8U(*isBlocked)[WIDTH], INT16U* n_route, const Pos start, const Pos arrive);

/*
struct List {
	INT8U* next;
	INT8U  data;
} List;

INT8U search_list(struct List* head, INT8U data);						
INT8U listSize(struct List* head);									// num of nodes except first node
void push_list(struct List* head, struct List* addOne);
INT8U pop_list(struct List* head, int n);							// return n th node's data (num n is except first node)
*/

#endif // _MY_UTIL_H_

