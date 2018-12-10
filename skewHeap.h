#include <iostream>
#include <stdlib.h>
#include "TreeBackTrack.h"
#include "HeapStat.h"

#include <tr1/memory>

template <typename T, class Less=std::less <T>, class StatEvent=heap_stat::NoOp<int> > class SkewHeap
{
    struct Node;
    typedef std::tr1::shared_ptr<struct Node> NodeSP;
	public:
    typedef NodeSP ElemHandle;
    private:
	struct Node {
	NodeSP firstRoot, secondRoot, parent;
	T key;
	ElemHandle *handle;
	
        bool similar(NodeSP other) {
	        return key == other->key; //:(
	        return other->firstRoot && other->firstRoot==firstRoot 
	        || other->secondRoot && other->secondRoot==secondRoot;
        }
        
        void print(int level) {
                for (int i = 0; i < level; i++) {
                    std::cout<<" ";
                }
                std::cout<<key<<std::endl;
                if (firstRoot != 0) {
                    firstRoot->print(level + 1);
                }
                if (secondRoot != 0) {
                    secondRoot->print(level + 1);
                }
            }

  template <typename OutputIt> void copy_lower(const T &bound, OutputIt  oi)
  {
	  if (Less()(key, bound)) {
		StatEvent()(heap_stat::COPY_LOWER);
		*oi++ = key;
		if (firstRoot) 
	  		firstRoot->copy_lower(bound, oi);
		if (secondRoot)
	  		secondRoot->copy_lower(bound, oi);
	} else
				StatEvent()(heap_stat::COPY_LOWER_OVERHEAD);
  }

bool childIdx(const NodeSP &child) const {
	if (!this || secondRoot == child)
		return true;
	else if (firstRoot == child)
		return false;
	else {
				std::cerr << "Internal error: Parent doesn't refer a child" <<std::endl;
				exit(2);
			}
}

		NodeSP mwCopy() {
			if (!this) return NodeSP();
			NodeSP n(new Node(*this));
			if (n->firstRoot)
				n->firstRoot->parent = n;
			if (n->secondRoot)
				n->secondRoot->parent = n;
			if (handle) *handle=n;
			return n;
		}

        Node(T _key, const NodeSP& _firstRoot, const NodeSP& _secondRoot, 
        	const NodeSP& _parent) :
        	firstRoot(_firstRoot), secondRoot(_secondRoot), parent(_parent), key(_key), handle(0) {}
        Node(T _key) :
        	key(_key), handle(0) {}
};
	
NodeSP root;
static NodeSP merge(const NodeSP &firstRoot, const NodeSP &secondRoot)
{
    if(firstRoot == NULL)
        return secondRoot;

    else if(secondRoot == NULL)
        return firstRoot;

    if(!Less()(secondRoot->key, firstRoot->key))
    {
        assert (Less()(firstRoot->key, secondRoot->key));//temporary
        NodeSP result(new Node(firstRoot->key, merge(secondRoot, firstRoot->secondRoot),
        	firstRoot->firstRoot,NodeSP() ) );
        if (result->firstRoot) result->firstRoot->parent = result;
        if (result->secondRoot) result->secondRoot->parent = result;
        if (firstRoot->handle) 
	        *(result->handle=firstRoot->handle)=result;
        StatEvent()(heap_stat::MERGE);
        return result; 
    }
    else
        return merge(secondRoot, firstRoot);
}

		static NodeSP mwNewParent(const NodeSP &newChild, bool isSecondChild) {
			if (!newChild->parent) //    ???
				return NodeSP();
			std::cout<<"New child:";
				newChild->print(0); std::cout<<"----"<<std ::endl;
			NodeSP newParent(newChild->parent->mwCopy());
			if (!isSecondChild) {
				newParent -> firstRoot = newChild;
			} else 
				newParent -> secondRoot = newChild;
			//now r points to this
			if (newParent->firstRoot) newParent->firstRoot->parent = newParent;
			if (newParent->secondRoot) newParent->secondRoot->parent = newParent;
			std::cout<<"New parent:";
				newParent->print(0); std::cout<<"----"<<std ::endl;
			return newParent;
		}
				
     static NodeSP mw_bubbleUp(NodeSP node, bool toRoot) {
        NodeSP old_node = node;
        node = node->mwCopy();
        NodeSP old_parent = old_node->parent;
        NodeSP parent = mwNewParent(node,old_parent->childIdx(old_node));
        while (parent != 0 && (toRoot || Less()(node->key, parent->key) ) ) {
            StatEvent()(heap_stat::PATH_TRAVERSAL);
            T temp = node->key;
            node->key = parent->key;
            parent->key = temp;
            ElemHandle *temphandle = node->handle;
            node->handle = parent->handle;
            parent->handle = temphandle;
            if (node->handle) * node->handle=node;
            if (parent->handle) * parent->handle=parent;

            node = parent->mwCopy();
            old_node = old_parent;
			old_parent = old_node->parent;
            parent = mwNewParent(parent, old_parent->childIdx(old_node));
        }
        std::cout<<" INFO: Detached:\n"; node->print(0); std::cerr<<std::endl;
        return (node);
    }

public:
crrc(NodeSP n) {
	T k = n->key;
	while (n->parent) n=n->parent;
	if (n != root) {
		std::cerr << "INTERNAL: CRRC FAILED FOR "<<k<<std::endl;
		exit(1);
	}
}

SkewHeap &operator |= (const SkewHeap &other)
{
	root = merge(root,other.root);
	return  *this;
}

    ElemHandle *insert(T key) {
        ElemHandle *h = new ElemHandle(new Node(key));
        (*h)->handle=h;
        *this |= SkewHeap(*h);
        return h;
    }

static SkewHeap erase_from_last_recent(ElemHandle *h) {
	NodeSP root = mw_bubbleUp(*h, true);
	if (root->firstRoot)
		root->firstRoot->parent.reset();
	if (root->secondRoot)
		root->secondRoot->parent.reset();
	return SkewHeap<T,Less,StatEvent>(merge(root->firstRoot, root->secondRoot));
}

SkewHeap() {}
SkewHeap(const NodeSP _root) 
	:root(_root) {}

  void print() {
        std::cout<<"Binomial heap:"<<std::endl;
        if (root != 0) {
            root->print(0);
        }
    }
    
  template <typename OutputIt> void copy_lower(const T &bound, OutputIt oi)
  {
	  if (root)
	  	root->copy_lower(bound, oi);
  }
  
    

};
