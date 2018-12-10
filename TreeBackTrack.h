#ifndef _TREE_BACKTRACK_
#define _TREE_BACKTRACK_
#include <tr1/memory>
template <class NodeP>
struct TreeBackTrack {
	typedef std::tr1::shared_ptr<TreeBackTrack> GroupSP;
	GroupSP parent;
	NodeP head;
	void  notice_head(NodeP h) {head = h;}
	void  notice_parent(const GroupSP &p) {parent = p;}
	
	NodeP getParentHead() {return parent? parent->head: 0;}
};
#endif //_TREE_BACKTRACK_