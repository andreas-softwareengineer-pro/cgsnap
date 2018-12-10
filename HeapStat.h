#ifndef _HEAP_STAT_H_
#define _HEAP_STAT_H_
namespace heap_stat {

    enum EventE {
	    CREATE_NODE,
	    CHILDREN_TRAVERSAL,
	    PATH_TRAVERSAL,
	    MERGE,
	    COPY_LOWER,
	    COPY_LOWER_OVERHEAD,
	    EVENT_KIND_COUNT //keep last
    };

template <typename T> struct NoOp {
	void operator()(const T &) {}
};

} //namespace heap_stat
#endif // _HEAP_STAT_H_