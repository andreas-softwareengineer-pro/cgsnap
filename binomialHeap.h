#include <tr1/memory>
#include <iostream>
#define NDEBUG 1
#include <cassert>
#include <deque>
#include "HeapStat.h"
#include "TreeBackTrack.h"

static int		curr_tour_stamp =  0;

#define BINOMIAL_HEAP_SANITY_CHECK

#ifdef BINOMIAL_HEAP_SANITY_CHECK
#define SANITY_CHECK(n) structSanityCheck(n);
#else
#define SANITY_CHECK(n)
#endif

#ifdef COUNT_HEAP_NODE_CONSTRUCTION
#define HEAP_NODE_COUNTED : private Count
#else //COUNT_HEAP_NODE_CONSTRUCTION
#define HEAP_NODE_COUNTED 
#endif

template <typename T, class Less=std::less <T>, class StatEvent=heap_stat::NoOp<int> > class BinomialHeap {
    struct Node;
    typedef std::tr1::shared_ptr<struct Node> NodeSP;

	public:
    typedef NodeSP ElemHandle;
    private:
    struct Node HEAP_NODE_COUNTED
        {
        T key;
        int degree;
        NodeSP parent;
        typename TreeBackTrack<NodeSP>::GroupSP group;
        NodeSP child;
        NodeSP sibling;//unshareable
        ElemHandle* handle; //
		int  tour_stamp;
//        Node() : key(......) {}

        Node(T _key) : key(_key), degree(0),  handle (0), tour_stamp(curr_tour_stamp) {}
		NodeSP mwCopy() {
			if (!this) return NodeSP();
			NodeSP n(new Node(*this)); n->tour_stamp = curr_tour_stamp;
			for (NodeSP r = child; r; r = r->sibling)
					r->parent = n; //Complexity???
			if (handle) *handle=n;
			return n;
		}
		

  template <typename OutputIt> void copy_lower (const T &bound, OutputIt  oi)
  {
		Node *curr = this;
		while (curr) {
			if (Less()(curr->key, bound)) {
				StatEvent()(heap_stat::COPY_LOWER); 
				*oi++ = key;
	  			if (curr->child)
	  				curr->child->copy_lower(bound, oi);
	} else
				StatEvent()(heap_stat::COPY_LOWER_OVERHEAD); 
	curr = curr->sibling.get();
		}
  }

        void print(int level) {
            Node* curr(this);
            while (curr != 0) {
                std::cout<<"["<<degree<<"]";
                for (int i = 0; i < level; i++) {
                    std::cout<<" ";
                }
                std::cout<<curr->key<<std::endl;
                if (curr->child != 0) {
                    curr->child->print(level + 1);
                }
                curr = curr->sibling.get();
            }
        }
    };

    static bool less(NodeSP node, NodeSP other) {
            return Less()(node->key,other->key);
    }
    NodeSP head;

    public:
//Sanity check
#ifdef BINOMIAL_HEAP_SANITY_CHECK
#define SANERR "Sanity error: "
	//1. Each node should only point to the same or earlier epoch nodes
	static void structSanityCheck(NodeSP& n)
	{
		if (!n) return;;
		if (n->child && n->child->tour_stamp > n->tour_stamp)
			{
				std::cerr << SANERR << n->key <<": " << "A child is newer than its parent" <<std::endl;
				abort();
			}
		if (n->sibling && n->sibling->tour_stamp > n->tour_stamp)
			{
				std::cerr << SANERR << n->key <<": " << "A sibling is newer than its parent" <<std::endl;
				n->sibling->print(0);
				abort();
			}
		structSanityCheck(n->child);
		structSanityCheck(n->sibling);
	}
#endif
    BinomialHeap() {}

    BinomialHeap(NodeSP head) {
        this->head = head;
    }

    bool isEmpty() {
        return head == 0;
    }

    void clear() {
        head = 0;
    }

    ElemHandle *insert(T key) {
        ElemHandle *h = new ElemHandle(new Node(key));
        (*h)->handle=h;
        (*h)->group.reset(new TreeBackTrack<NodeSP>());
        (*h)->group->notice_head(head);
        *this |= BinomialHeap(*h);
        return h;
    }

    T findMinimum() {
        if (!head) {
            return -1; //??
        } else {
            NodeSP min = head;
            NodeSP next = min->sibling;

            while (next) {
                if (less(next,min)) {
                    min = next;
                }
                next = next->sibling;
            }

            return min->key;
        }
    }

    // Implemented to test delete/decrease key, runs in O(n) time
    NodeSP search(T key) {
        std::deque<NodeSP> nodes;
        nodes.add(head);
        while (!nodes.isEmpty()) {
            NodeSP curr = nodes.get(0);
            nodes.remove(0);
            if (curr->key == key) {
                return curr;
            }
            if (curr->sibling != 0) {
                nodes.add(curr->sibling);
            }
            if (curr->child != 0) {
                nodes.add(curr->child);
            }
        }
        return 0;
    }

		static NodeSP mwNewParent(const NodeSP &newChild) {
			if (!newChild->parent 
			|| !newChild->parent->child) //    ???
				return NodeSP();
			std::cout<<"New child:";
				newChild->print(0); std::cout<<"----"<<std ::endl;
			NodeSP newParent(new Node(*newChild->parent));
			NodeSP *prev_p = &newParent->child;
			Node* r = newChild->parent->child.get();
			for (; r && r->sibling!=newChild->sibling; r = r->sibling.get()) {
				*prev_p = r->mwCopy();
				prev_p = &(*prev_p)->sibling;
			}
			if (!r) {
				std::cerr << "Internal error: Parent doesn't refer a child" <<std::endl;
				return NodeSP();
			}
			//now r points to this
			newChild->sibling=(*prev_p)->sibling;
			*prev_p = newChild;
			//reparent all (both copied and old if any) children
			for (NodeSP r = newChild; r; r = r->sibling) //This is of critical complexity - rearrange
					r->parent = newParent;
			std::cout<<"New parent:";
				newParent->print(0); std::cout<<"----"<<std ::endl;
			return newParent;
		}
				
    void decreaseKey(NodeSP node, T newKey) {
        node->key = newKey;
        bubbleUp(node, false);
    }

    static BinomialHeap erase_from_last_recent(ElemHandle *h) {
        NodeSP node = mw_bubbleUp(*h, true);
        BinomialHeap heap; //???
        if (heap.head->sibling == node->sibling) {
            heap.head = node;
            heap.removeTreeRoot(node, 0);
        } else {
            NodeSP *prev = &(heap.head = heap.head->mwCopy())->sibling;
            //while (less(prev->sibling,node)) {
            while ((*prev)->sibling != node->sibling) {
                (*prev) = (*prev)->mwCopy();
                prev = &(*prev)->sibling;
            }
            heap.removeTreeRoot(node, prev);
        }
        return heap;
    }


    private:
     NodeSP bubbleUp(NodeSP node, bool toRoot) {
        NodeSP parent = node->parent;
        while (parent != 0 && (toRoot || less(node, parent) ) ) {
            StatEvent()(heap_stat::PATH_TRAVERSAL);
            T temp = node->key;
            node->key = parent->key;
            parent->key = temp;
            node = parent;
            parent = parent->parent;
        }
        return node;
    }

     static NodeSP mw_bubbleUp(NodeSP node, bool toRoot) {
        node = node->mwCopy();
        NodeSP parent = mwNewParent(node);
        while (parent != 0 && (toRoot || less(node, parent) ) ) {
            T temp = node->key;
            node->key = parent->key;
            parent->key = temp;
            ElemHandle *temphandle = node->handle;
            node->handle = parent->handle;
            parent->handle = temphandle;
            if (node->handle) * node->handle=node;
            if (parent->handle) * parent->handle=parent;

            node = parent;
            parent = mwNewParent(parent);
        }
        std::cout<<" INFO: Detached:\n"; node->print(0); std::cerr<<std::endl;
        return node;
    }

    public:
    T extractMin() {
        if (head == 0) {
            return 0;
        }

        NodeSP min = head;
        NodeSP *minPrev = 0;
        NodeSP next = min.sibling;
        NodeSP nextPrev = min;

        while (next != 0) {
            if (next.compareTo(min) < 0) {
                min = next;
                minPrev = &nextPrev->sibling;
            }
            nextPrev = next;
            next = next.sibling;
        }

        removeTreeRoot(min, minPrev);
        return min.key;
    }

    private:
    void removeTreeRoot(NodeSP root, NodeSP *prev_p) {
        // Remove root from the heap
        if (root == head) {
            head = root->sibling;
            if (head) head->group->notice_head(head);
        } else {
            *prev_p = root->sibling;
        }

        // Reverse the order of root's children and make a new heap
        NodeSP newHead;
        typename TreeBackTrack<NodeSP>::GroupSP newGroup(new TreeBackTrack<NodeSP>());

        NodeSP child = root->child->mwCopy();
        while (child != 0) {
            NodeSP newChild = child->sibling->mwCopy();
            child->sibling = newHead;
            child->parent.reset ();
            child->group = newGroup;
            newHead = child;
            child = newChild;
        }
        newGroup->notice_head(head);

        std::cout << "After remove: ";  head->print(0); std::cout<<std::endl;
        std::cout << "Addee: ";  newHead->print(0); std::cout<<std::endl;
        BinomialHeap<T,Less,StatEvent> newHeap(newHead);

        // Union the heaps and set its head as this.head
        (*this) |= newHeap;
        std::cout << "After merge: ";  head->print(0); std::cout<<std::endl;
        
    }

    // Merge two binomial trees of the same order
    void linkTree(NodeSP minNodeTree, NodeSP other) {
	    assert(minNodeTree->tour_stamp == curr_tour_stamp);
	    assert(minNodeTree->other == curr_tour_stamp);
        other->parent = minNodeTree;
        if (minNodeTree->child) other->group = minNodeTree->child->group;
        other->group -> notice_parent(minNodeTree->group);
        other->sibling = minNodeTree->child;
        minNodeTree->child = other;
        minNodeTree->degree++;
    }

    public:
    // Union two binomial heaps into one and return the head
    BinomialHeap& operator |= (const BinomialHeap& heap) {
        curr_tour_stamp++;
        NodeSP newHead = merge(*this, heap);

        head = NodeSP();
        //heap.head = NodeSP();

        if (newHead == 0) {
            return *this;
        }

        NodeSP prev;
        NodeSP curr = newHead;
        NodeSP next = newHead->sibling;

        while (next != 0) {
            if (curr->degree != next->degree || (next->sibling != 0 &&
                    next->sibling->degree == curr->degree)) {
                prev = curr;
                curr = next;
            } else {
                if (less(curr,next)) {
                    curr->sibling = next->sibling;
                    linkTree(curr, next);
                } else {
                    if (prev == 0) {
                        newHead = next;
                    } else {
                        prev->sibling = next;
                    }

                    linkTree(next, curr);
                    curr = next;
                }
            }

            next = curr->sibling;
        }

        SANITY_CHECK(head);
        head = newHead;
        SANITY_CHECK(head);
        if (head) head->group->notice_head(head);
	    StatEvent()(heap_stat::MERGE);
        return *this;             }

    private:
    static NodeSP merge(
            BinomialHeap<T,Less,StatEvent> heap1, const BinomialHeap<T,Less,StatEvent> heap2) {
        if (heap1.head == 0) {
            return heap2.head;
        } else if (heap2.head == 0) {
            return heap1.head;
        } else {
            NodeSP head;
            NodeSP heap1Next = heap1.head;
            NodeSP heap2Next = heap2.head;

            for (NodeSP h2 = heap2.head; h2; h2=h2->sibling)
            	h2->group = heap1.head->group;
        
            if (heap1.head->degree <= heap2.head-> degree) {
                head = heap1.head;
                heap1Next = heap1Next->sibling;
            } else {
                head = heap2.head;
                heap2Next = heap2Next->sibling;
            }

            NodeSP tail = head = head -> mwCopy();

            while (heap1Next != 0 && heap2Next != 0) {
                if (heap1Next->degree <= heap2Next->degree) {
                    tail->sibling = heap1Next -> mwCopy();
                    heap1Next = heap1Next->sibling;
                } else {
                    tail->sibling = heap2Next -> mwCopy();
                    heap2Next = heap2Next->sibling;
                }

                tail = tail->sibling;
            }

            //A- 
           /* if (heap1Next != 0) {
                tail->sibling = heap1Next;
            } else {
                tail->sibling = heap2Next;
            }
            A+ */
            while (heap1Next != 0) {
                tail->sibling = heap1Next->mwCopy();
                tail = tail->sibling;
                    heap1Next = heap1Next->sibling;
            }
            
            while (heap2Next != 0) {
                tail->sibling = heap2Next->mwCopy();
                tail = tail->sibling;
                    heap2Next = heap2Next->sibling;
            }
            
  	        head->group->notice_head(head);    
            return head;
        }
    }

    public:
    void print() {
        std::cout<<"Binomial heap:"<<std::endl;
        if (head != 0) {
            head->print(0);
        }
    }
    template <typename OutputIt> void copy_lower (const T &bound, OutputIt oi)
  {
	  head->copy_lower(bound, oi);
  }
    

}
;