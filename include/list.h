/*
Copyright (c) 2015, Intel Corporation. All rights reserved.
*Redistribution and use in source and binary forms, with or without
*modification, are permitted provided that the following conditions are met:
*
*1. Redistributions of source code must retain the above copyright notice,
*this list of conditions and the following disclaimer.
*
*2. Redistributions in binary form must reproduce the above copyright notice,
*this list of conditions and the following disclaimer in the documentation
*and/or other materials provided with the distribution.
*
*3. Neither the name of the copyright holder nor the names of its contributors
*may be used to endorse or promote products derived from this software without
*specific prior written permission.
*
*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
*AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
*ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
*LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
*ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef _LIST_H
#define _LIST_H

#include<stdio.h>
#include<stdlib.h>


#define _INLINE_ static inline

struct list_head {
    struct list_head *next, *prev;
};
#define LIST_HEAD_INIT(name) {&(name), &(name)}
#define LIST_HEAD(name) struct list_head name = LIST_HEAD_INIT(name)

#define INIT_LIST_HEAD(ptr) do { \
    (ptr)->next = (ptr); \
    (ptr)->prev = (ptr); \
} while (0)

_INLINE_ void __list_add(struct list_head *new,
	struct list_head *prev,
	struct list_head *next)
{
    next->prev = new;
    new->next = next;
    new->prev = prev;
    prev->next = new;
}

/*每次添加节点到head之后，始终都是添加到头结点之后*/
_INLINE_ void list_add(struct list_head *add, struct list_head *head){
    __list_add(add, head, head->next);
}

/*每次添加节点都是头结点之前，由于是循环链表，就是说添加到链表尾部*/
_INLINE_ void list_add_tail(struct list_head *add, struct list_head *head)
{
    __list_add(add, head->prev, head);
}

_INLINE_ void __list_del(struct list_head *prev, struct list_head *next)
{
    next->prev = prev;
    prev->next = next;
}

/*删除节点*/
_INLINE_ void list_del(struct list_head *entry)
{
    __list_del(entry->prev, entry->next);
}

/*删除节点，并初始化被删除的结点（也就是使被删除的结点的prev和next都指向自己）*/
_INLINE_ void list_del_init(struct list_head *entry)
{
    __list_del(entry->prev, entry->next);
    INIT_LIST_HEAD(entry);
}

/*判断链表是否为空*/
_INLINE_ int list_empty(struct list_head *head)
{
    return head->next == head;
}

/*通过两个链表的head，进行连接*/
_INLINE_ void list_splice(struct list_head *list, struct list_head *head)
{
    struct list_head *first = list->next;
    if (first != list) {
	struct list_head *last = list->prev;
	struct list_head *at = head->next;
	first->prev = head;
	head->next = first;
	last->next = at;
	at->prev = last;
    }
}

#define list_entry(ptr, type, member) \
    ((type *)((char *)(ptr) - (unsigned long)(&((type *)0)->member)))

/*遍历链表，此时删除节点的操作可能会出错*/
#define list_for_each(pos, head) \
for (pos = (head)->next; pos != (head); pos = pos->next) //新代码中出现prefetch() 可以不考虑，用于预取以提高遍历速度

/*遍历链表，可以同时有删除节点的操作*/
#define list_for_each_safe(pos, pnext, head) \
for (pos = (head)->next, pnext = pos->next; pos != (head); \
	pos = pnext, pnext = pos->next)
#undef _INLINE_
#endif
