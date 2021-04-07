#include "my_util.h"

Pos* setShelfPos(Pos* ShelfPos) 
{
	int c = 0;

	for (INT8U j = 0; j < 7; j++) {
		for (INT8U i = 0; i < 7; i++) {
			ShelfPos[c++] = ij2Pos(4 + i, 2 + j * 5);
			ShelfPos[c++] = ij2Pos(4 + i, 3 + j * 5);
			ShelfPos[c++] = ij2Pos(14 + i, 2 + j * 5);
			ShelfPos[c++] = ij2Pos(14 + i, 3 + j * 5);
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

void pushPQ(PQ* root, Pos p1, INT16U w1)
{
	INT16U idx, w2;;

	root->cnt += 1;
	root->pos[root->cnt] = p1;
	root->weight[root->cnt] = w1;

	idx = root->cnt;
	while (idx > 0) {
		w1 = root->weight[idx];
		w2 = root->weight[idx >> 1];
		if (w1 < w2) {
			swap(root, idx/2, idx);
			idx = idx >> 1;	
		}
		else break;
	}
	
	return;
}

Pos popPQ(PQ* root) 
{
	Pos rst = root->pos[1];
	INT16U w, w1, w2;
	INT16U d;
	INT16U idx = root->cnt;

	swap(root, 1, idx);
	root->cnt -= 1;

	while (d = (idx*2 - root->cnt) >= 0) {
		if (d == 0) {
			w  = root->weight[idx];
			w1 = root->weight[idx * 2];
			if (w > w1) swap(root, idx, idx * 2);
			break;
		}
		else {
			w  = root->weight[idx];
			w1 = root->weight[idx * 2];
			w2 = root->weight[idx * 2 + 1];
			if (w1 < w2) {
				if (w > w1) {
					swap(root, idx, idx * 2);
					idx = idx * 1;
				}
				else break;
			}
			else {
				if (w > w2) {
					swap(root, idx, idx * 2 + 1);
					idx = idx * 2 + 1;
				}
				else break;
			}
		}
	}

	return rst;
}

void swap(PQ* root, INT16U idx1, INT16U idx2) 
{
	Pos t_p = root->pos[idx1];
	INT16U t_w = root->weight[idx1];

	root->pos[idx1] = root->pos[idx2];
	root->weight[idx1] = root->weight[idx2];

	root->pos[idx2] = t_p;
	root->weight[idx2] = t_w;

	return;
}

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

/*
INT8U search_list(struct List* head, INT8U data) 
{
	int c = 0;
	while (head != NULL) {
		if (head->data == data) {
			c += 1;
			return c;
		}
		head = head->next;
	}
	return c;
}

INT8U listSize(struct List* head) 
{
	int c = -1;
	while (head != NULL) {
		c += 1;
		head = head->next;
	}

	return c;
}

void push_list(struct List* head, struct List* addOne) 
{
	List* tmp = head->next;
	head->next = addOne;
	addOne->next = tmp;

	return;
}

INT8U pop_list(struct List* head, int n) 
{
	List* rst;
	for (int i = 0; i < n - 1; i++) {
		head = head->next;
	}
	rst = head->next;
	head->next = rst->next;
	rst->next = NULL;

	return rst->data;
}
*/