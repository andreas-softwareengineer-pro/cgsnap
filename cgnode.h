#ifndef _CGNODE_H_
#define _CGNODE_H_
#include <iterator>     // std::ostream_iterator
#include <ostream>
#include <vector>
#include <set>
#include "binomialHeap.h"
#include "skewHeap.h"
template <template<typename...> class H> struct CountHeapEvents {
	static int counts[heap_stat::EVENT_KIND_COUNT];
	void operator()(heap_stat::EventE e) {++counts[e];}
}
;
template <template<typename...> class H> 
int CountHeapEvents<H>::counts[heap_stat::EVENT_KIND_COUNT];

#define HEAP_TYPE(templ_name) templ_name <CGNode*,CGNode::LessStartNo,struct CountHeapEvents<templ_name> >
#define FOR_HEAP_TYPES(fld_base,rec) \
	rec(BinomialHeap,	fld_base##_bn) \
	rec(SkewHeap,		fld_base##_sk)
#define DEF_CONE_INFO_FIELD(templ_name,field)   ConeInfo<HEAP_TYPE(templ_name)> field;
	
class CGLessStartNo;
typedef uint64_t TargetVector, TerminalVector; //Initialize!!

struct CGNode;

template<class H> struct ConeInfo {
	H view;
	std::vector<typename H::ElemHandle *> fires;
	typename H::ElemHandle *handle;
	ConeInfo() :handle(0) {}
};


struct CGSCC;
struct CGNode {
	typedef uint64_t TermMask;
	std::string name;
	std::vector<CGNode*> outgoing;
	struct FuncInfo *func;
	int start_no, end_no;
	bool visited;
	bool has_return;
	unsigned int fill;
	std::string catches, throws;
	std::vector<struct FuncInfo*> callsResolved;
	CGSCC* scc;
	int scc_low; //no need to initialize
	struct LLAssign* calls;
	CGNode* unwind, *binary_hint, *quasibinary_hint;
	TargetVector x, cone_x;
	TerminalVector y, cone_y;
	struct LessStartNo	{
		bool operator()(CGNode *x, CGNode *y) {return x->start_no < y->start_no;}
	};
	FOR_HEAP_TYPES(coneOutgoing,DEF_CONE_INFO_FIELD)
	CGNode *lastSeenOugoingFrom;
	std::vector<CGNode*> ancestors;

	void setReturn() {has_return = true;}
	CGNode(FuncInfo* _func) : func(_func), visited(false),  has_return(false),
		fill(0), calls(0), unwind(0), binary_hint(0), quasibinary_hint(0), lastSeenOugoingFrom(0)   {}
	CGNode(FuncInfo* _func, const std::string &_name) : name( _name ), func(_func), visited(false), has_return(false),
		fill(0), calls(0), unwind(0), binary_hint(0), quasibinary_hint(0), lastSeenOugoingFrom(0)   {}
};

struct CGSCC {
	std::vector<CGNode*> nodes;
	CGNode::TermMask y;
	bool y_ready;
	CGSCC(struct FuncInfo *_function) : y_ready(false) {}
};

struct FuncInfo {
	CGNode* entry; //entry [0] is the function entry, 1.. are catch-"enclaves"
	std::string name;
	bool internal;
	std::map<std::string, int> throws;
	std::vector<CGNode*> invokedFrom;
	std::vector<CGNode*> nodes;
	CGNode *quasibinaryTree, *binaryTree;

	FuncInfo(const std::string &_name, bool _internal)
	: entry(0), name(_name), internal(_internal), quasibinaryTree(0), binaryTree(0) {}
};

void addEdge(CGNode* f, CGNode* t)
{
	f->outgoing.push_back(t);
	std::cout << f->name << " -> " << t->name << std::endl;
};

	template<typename T>
	std::ostream &operator << (std::ostream &o, const std::vector<T>& v)
	{
		for (auto & x : v)
			o << x << " ";
		return o;
	}
	template<typename T>
	std::ostream &operator << (std::ostream &o, const std::set<T>& v)
	{
		for (auto & x : v)
			o << x << " ";
		return o;
	}
	std::ostream &operator << (std::ostream &o, CGNode *n)
	{
		if (n)
			o << n->func->name << ":" << n->name;
		else o << "NULL";
		return o;
	}

	std::ostream &operator << (std::ostream &o, FuncInfo *f)
	{
		if (f)
			o << f->name;
		else o << "NULL";
		return o;
	}

	std::ostream &operator << (std::ostream &o, CGNode &n)
	{
		o << "Node " << n.func << ":" << n.name << "["<< n.start_no << "-"<< n.end_no << "]"<<std::endl;
		o << "Outgoing " << n.outgoing << std::endl;
#	define PRINT_O(tn,f) o << "Cone out @" << #tn << ": "; \
	n.f.view.copy_lower(&n,std::ostream_iterator<CGNode*>(o, " ")); \
	o << std::endl;
	FOR_HEAP_TYPES(coneOutgoing,PRINT_O)
#	undef PRINT_O

	if (n.has_return) o << "Return" << std::endl;
	o << "Fill: " << n.fill <<std::endl;
	if (!n.catches.empty()) o << "Catches "<< n.catches <<std::endl;
	if (!n.throws.empty()) o << "Throws "<< n.throws <<std::endl;
	if (!n.callsResolved.empty()) o << "Calls " << n.callsResolved <<std::endl;
	if (n.scc)
		o << "SCC no. " << n.scc_low <<std::endl; //no need to initialize
	//if (!n.calls.empty()) o << "Calls (raw) " << n.calls << std::endl;
	if (n.unwind) o << " Unwind to " << n.unwind << std::endl;
	//FOR_HEAP_TYPES(coneOutgoing,DEF_CONE_INFO_FIELD)
	//CGNode *lastSeenOugoingFrom;
		return o;
	}

	std::ostream &operator <<(std::ostream &o, const FuncInfo &f) {
		if (f.internal) o<<"static ";
		o << f.name << std::endl;
		o << "Invoked from " << f.invokedFrom << std::endl;
		o << "================================" << std::endl;;
		for (auto n: f.nodes) o<<*n<<"-----------------------" << std::endl;
		o << "================================" << std::endl;;
		return o;
	}

#endif