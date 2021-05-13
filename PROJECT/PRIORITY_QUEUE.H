#include "my_util.h"

/*
*********************************************************************************************************
*                                             Initialize Map
*********************************************************************************************************
*/

Pos* setShelfPos(Pos* ShelfPos) 
{
	int c = 0;

	for (INT8U j = 0; j < 7; j++) {
		for (INT8U i = 0; i < 7; i++) {
			ShelfPos[c++] = ij2Pos(3 + i, 2 + j * 5);
			ShelfPos[c++] = ij2Pos(3 + i, 3 + j * 5);
			ShelfPos[c++] = ij2Pos(13 + i, 2 + j * 5);
			ShelfPos[c++] = ij2Pos(13 + i, 3 + j * 5);
		}
	}

	return ShelfPos;
}

Pos* setParkingPos(Pos* ParkingPos, INT16U num) 
{
	INT8U i, n = num >> 1;
	struct Pos p;

	for (i = 0; i < n; i++) 
		ParkingPos[i] = ij2Pos(0, 2 + 4 * i);
	
	for (i = n; i < num; i++) 
		ParkingPos[i] = ij2Pos(22, 1 + 4 * i);
	
	return ParkingPos;
}

Pos* setTPPos(Pos* TPP, INT16U num) 
{
	for (INT8U i = 0; i < num; i++) 
		TPP[i] = ij2Pos(0, 1 + 4 * i);

	return TPP;
}

Pos* setLDPos(Pos* LDP, INT16U num) 
{
	for (INT8U i = 0; i < num; i++)
		LDP[i] = ij2Pos(22, 2 + 4 * i);

	return LDP;
}

void setblockedPos(INT8U(*blockedPos)[WIDTH], const Pos* ShelfPos, INT16U num) 
{
	memset(blockedPos, 0, HEIGHT * WIDTH);

	for (INT8U i = 0; i < num; i++) 
		blockedPos[ShelfPos[i].i][ShelfPos[i].j] = 1;

	for (INT8U i = 0; i < 9; i++) {
		blockedPos[0][0 + 4 * i] = 1;
		blockedPos[0][3 + 4 * i] = 1;
		blockedPos[22][0 + 4 * i] = 1;
		blockedPos[22][3 + 4 * i] = 1;
	}

	return;
}

__inline INT16U distance(Pos p1, Pos p2)
{
	return (ABS(p1.i - p2.i) + ABS(p1.j - p2.j));
}

__inline Pos ij2Pos(INT8U i, INT8U j) 
{
	Pos p;
	p.i = i; p.j = j;
	return p;
}

/*
*********************************************************************************************************
*                                             AStarSearch
*********************************************************************************************************
*/

void AStarSearch(Pos* route, const INT8U (*isBlocked)[WIDTH], INT16U* n_route, const Pos start, const Pos arrive) 
{
	PQ pq;
	Pos next, tmp;
	Pos camefrom[HEIGHT][WIDTH];
	INT16U weight[HEIGHT][WIDTH];
	INT16U t_w;
	INT8U i, j;
	n_route = 0;

	// 거꾸로 도착점에서 부터 시작함
	pushPQ(&pq, arrive, distance(start, arrive));
	weight[arrive.i][arrive.j] = distance(start, arrive);
	while (1) {
		next = popPQ(&pq);
		if (next.i == start.i && next.j == start.j) break;
		t_w = weight[next.i][next.j];
		i = next.i;
		j = next.j;

		if (i - 1 >= 0 && !isBlocked[i-1][j]) {
			tmp = ij2Pos(i - 1, j);
			pushPQ(&pq, tmp, t_w + distance(start, tmp) + 1);
		} 
		if (j - 1 >= 0 && !isBlocked[i][j-1]) {
			tmp = ij2Pos(i, j - 1);
			pushPQ(&pq, tmp, t_w + distance(start, tmp) + 1);
		}
		if (i + 1 <= HEIGHT && !isBlocked[i+1][j]) {
			tmp = ij2Pos(i + 1, j);
			pushPQ(&pq, tmp, t_w + distance(start, tmp) + 1);
		}
		if (j + 1 <= WIDTH && !isBlocked[i][j+1]) {
			tmp = ij2Pos(i, j + 1);
			pushPQ(&pq, tmp, t_w + distance(start, tmp) + 1);
		}
	}

	// 거꾸로 도착점에서 부터 시작함
	tmp = start;
	while (!(tmp.i == arrive.i && tmp.j == arrive.j)) {
		route[*n_route++] = tmp;
		tmp = camefrom[tmp.i][tmp.j];
	}
	route[*n_route++] = arrive;
	
	return;
}

INT8U isBlockedPos(Pos p) {
	static const Pos* blockPos_ptr;
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
*                                                Others
*********************************************************************************************************
*/

INT8U getIdleStuff(INT32U* idle_code, INT8U idle_bound, INT8U idx)
{
	INT8U c = 0;

	while ((*idle_code & (1 << idx)) == 0) {
		if (++idx == idle_bound) idx = 0;
		if (c++ == idle_bound) return -1;
	}

	*idle_code -= (1 << idx);			// 유휴상태 정보 갱신
	return idx;
}

INT16U getIdleShelf(INT32U* idle_shelf, INT8U idle_bound, INT8U idx)
{
	INT8U i = idx / 32, j = idx % 32;

	while ((idle_shelf[i] & (1 << j)) == 0) {
		if (++j == 32) {
			if (++i == idle_bound / 32 + 1) i = 0;
			j = 0;
		}
	}

	idle_shelf[i] -= (1 << j);		// 유휴상태 정보 갱신

	return i * 32 + j;
}
