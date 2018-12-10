# ifndef __MARK_INTEREST_H_
# define __MARK_INTEREST_H_
# include "linked.h"

template <class Obj>
class MarkInterest {
	public:
	struct Edge {
		Obj *dst, *src;
		Edge *next;
		Edge(Obj *s, Obj *d): dst(d), src(s), next(0) {}
	};
	private:
	Linked<Edge,&Edge::next> queue, erase_queue;
	friend int checkTotal();
	void handle(Edge *e, bool okToErase) {
		if (e->src->ownerLive())
			if (e->dst->ownerLive() && e->src->owner!=e->dst->owner)
			{
					std::cerr << "Live>live append" << std::endl;
					e->dst->owner->boundary.append(e);
			} else {
					if (!e->dst->ownerLive())
						e->dst->interesting = true;
					if (e->dst->owner != e->src->owner)
						{
							e->dst->owner = e->src->owner;
							std::cerr << "Live>dead mvappend" << std::endl;
							queue.mvAppend(e->dst->outgoing);
						}
					e->src->outgoing.append(e);
				}
		else
				if (!e->dst->ownerLive())
					if (okToErase)
					{
						if (okToErase && e->dst->interesting)
						{
							e->dst->interesting = false;
							std::cerr << "Dead>dead erase" << std::endl;
							erase_queue.mvAppend(e->dst->outgoing);
						}
						e->src->outgoing.append(e);
					}
					else {
						erase_queue.append(e);
						std::cerr << "Dead>dead erappend" << std::endl;
						}	
				else {
					queue.mvAppend(e->src->outgoing);
						std::cerr << "Dead>live reclaimed" << std::endl;
					e->src->outgoing.append(e);
				}
				
	}
	
  public:
  	int markInterest(int maxSteps) {
		int step = 0;
		while (step < maxSteps) {
			if (!queue.empty())
				handle(queue.rmFirst(), false);
			else if (!erase_queue.empty())
				handle(erase_queue.rmFirst(), true);
			else
				return step;
			++step;
		}
		return step;
	}
	
	void insert(Obj *src, Obj *dst)
	{
		queue.append(new Edge(src, dst));
	}
	
	void enqueue(Obj *src)
	{
		queue.mvAppend(src->outgoing);
		if (!src->interesting)
			queue.mvAppend(src->boundary);
	}

	};
	
# endif //__MARK_INTEREST_H_