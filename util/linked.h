# ifndef __LINKED_H_
# define __LINKED_H_

template <class Obj, Obj * Obj::* next>
struct Linked {
	Obj *first;
	Obj **lastP;

	Linked() {
		lastP = &first;
		first = 0;
	}
	
	bool empty() {
		return !first;
	}
	
	void mvAppend(Linked &other) {
		if (other.first)
		{
					*lastP = other.first;
					lastP = other.lastP;
		}
		other.first = 0;
		other.lastP = &other.first;//ripped
	}
	Obj* rmFirst() {
		if (!first) return 0;
		Obj *rc = first;
		first = first->next;
		if (!first) lastP = &first;
		rc -> next = 0;
		return rc;
	}
	void append(Obj *other) {
		assert(!other->next);
		*lastP = other;
		lastP = &other->next;
	}
	int countsize() {
		int k = 0;
		Obj* p = first;
		while (p) ++k, p = p->next;
		return k;
	}
		
};
# endif
