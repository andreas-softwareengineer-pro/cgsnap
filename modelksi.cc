#include <math.h>
#include <string>
#include <string.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <memory>
#include <assert.h>
#include <map>
#include <unordered_map>
#include "ranges.h"
#include "cgnode.h"
#include "lexpattern.h"
#include "complexity_stat.h"

typedef unsigned int PM; //plus minus -- move to CGNODE?
#define FIELD_NAME_STR_E(f,t) #f,
const char *heapTypeStr[] = {FOR_HEAP_TYPES(,FIELD_NAME_STR_E) 0};
#undef FIELD_NAME_STR_E


struct ModelMode {
	bool use_cone_outgoing;
	bool use_binary_outgoing;
	bool use_quasibinary_outgoing;
	enum QueueMode {
		STACK,
		FIFO,
		SIZE_PRIORITY,
		RANDOM } queue_mode;
#	define HEAP_E(t,field) t,
	enum HeapType {
		FOR_HEAP_TYPES(,HEAP_E)
		LAST_HEAP_TYPE
	} heap_type;
	
	static std::vector<ModelMode> allModes() {
		std::vector<ModelMode> rc(2*3*LAST_HEAP_TYPE);
		for (bool uco : {true,false})
		for (bool ubo : {true,false})
		for (bool uqbo :{true,false}) if (!ubo || !uqbo)
		for (HeapType ht : {
					FOR_HEAP_TYPES(,HEAP_E)
				}) if (ht != LAST_HEAP_TYPE)
		for (auto qm : {STACK,FIFO,SIZE_PRIORITY,RANDOM})
				rc.push_back(ModelMode{uco,ubo,uqbo,qm,ht});
		return rc;
	}
	
	bool binary_outgoing_available() const {return use_binary_outgoing;}
	bool cone_outgoing_available() const {return use_cone_outgoing;}
	bool quasibinary_outgoing_available() const {return use_quasibinary_outgoing;}
	std::string getDesc() const {
		std::stringstream ss;
		if (use_cone_outgoing) ss<<"cone_";
		if (use_binary_outgoing) ss<<"binary_";
		if (use_quasibinary_outgoing) ss<<"quasibinary_";
		if (use_cone_outgoing) ss<<heapTypeStr[heap_type];
		static const char * queueModeNames [] = {"stack","fifo","size_priority","random"};
		ss<<queueModeNames[queue_mode];
		return ss.str();
	}	
#	undef HEAP_E
};

enum CallKind {CALL, INLINED};

class Queue {
	std::deque<CGNode* > d;
	struct LessSize {
		bool operator()(const CGNode *x, const CGNode *y) {
			return x-> end_no - x-> start_no < y-> end_no - y-> start_no;		
		}
	};
	struct RandomCmp {
		bool operator()(const CGNode *, const CGNode *) {
			return rand()&4;		
		}
	};
		
	public:
	void push(CGNode *n, ModelMode::QueueMode mode) {
			switch (mode) {
				case ModelMode::FIFO:
					d.push_front(n);
					break;
				case ModelMode::STACK:
					d.push_back(n);
					break;
				case ModelMode::SIZE_PRIORITY:
					d.push_back(n);
					break;
					std::make_heap(d.begin(),d.end(),LessSize());
				case ModelMode::RANDOM:
					d.push_back(n);
					std::make_heap(d.begin(),d.end(),RandomCmp());
					break;
		}		
	}

	CGNode *remove(ModelMode::QueueMode mode) {
			if (d.empty()) return 0;
			switch (mode) {
				case ModelMode::SIZE_PRIORITY:
					std::pop_heap(d.begin(),d.end(),LessSize());
					break;
				case ModelMode::RANDOM:
					std::pop_heap(d.begin(),d.end(),RandomCmp());
					break;
				default:
					break;
			}
			CGNode  *rc = d.back();
			d.pop_back();
			return rc;	
	}
};
	
template<class F> void dfs_e(CGNode* n, const F& f)
{
	if (!n->visited) {
		n->visited = true;
		for (auto &t : n-> outgoing) {
			f(n,t);
			dfs_e(t, f);
		}
	}
}

bool regularEdge(CGNode *n, int idx)
{
	return n->catches.empty() || idx  > 0; //Otherwise a catch go edge 
}

void clear_visit(CGNode* n)
{
	if (n->visited) {
		n->visited = false;
		for (auto &t : n-> outgoing)
			clear_visit(t);
	}
}

void clear_visit_invoked_from(CGNode* n)
{
	if (n->visited) {
		n->visited = false;
		for (auto &t : n-> outgoing)
			clear_visit(t);
		for (auto &t : n-> func-> invokedFrom)
			clear_visit(t);
	}
}

void sort_nodes(CGNode* n, int &j, std::vector<CGNode*> *nodevec)
{
	if (!n->visited) {
		n->visited = true;
		n->start_no = j++;
		if (nodevec) nodevec->push_back(n);
		for (size_t  ti = 0U;  ti < n-> outgoing.size(); ++ti)
			if (regularEdge(n,ti))
				//"regular" edge
				sort_nodes(n->outgoing[ti],j,nodevec);
//			else
				//catch-go node isn't part of our cone by default.
				//Add it to entries vector to make an enclave if needed
//				n->func->entry.push_back(n->outgoing[ti]);
		n->end_no = j;
	}
}

enum AssignKind {
	A_GENERAL,
	A_VAR,
	A_ES_SELECTOR,
	A_ES_MATCH
};

struct LLAssign {
	AssignKind kind;
	std::string s;
	std::vector<LLAssign*> deps;
	LLAssign(AssignKind _kind, const std::string&  _s) :kind( _kind),s(_s) {}
	LLAssign(const std::vector<LLAssign*> _deps) :kind(A_GENERAL),deps(_deps) {}
};

std::map<std::string,LLAssign*> var;
LLAssign* substituteIfPossible(const std::string &x){
	auto xi = var.find(x);
	return xi != var.end()? xi->second : new LLAssign(A_VAR,x);
}

void  collectVars(const LLAssign *a, std::set<std::string> &out)
{
	if (!a)
		return;
	for (LLAssign *child : a->deps)
		collectVars(child, out);
	if (a->kind == A_VAR)
		out.insert(a->s);
}

class NodeTracker {
	std::map<std::string,FuncInfo> &functions;
	std::map<std::string,CGNode*> blocks; //only valid while a given fucnction is processed
	CGNode *currNode;
	FuncInfo *currFunc;
	
public:

void def_fun_head(const std::string & funcName, bool internal)
{
	blocks.clear();
	currFunc = &functions.emplace(std::piecewise_construct,
	 std::make_tuple(funcName), 
	 std::make_tuple(
			funcName,
			internal))
	.first->second;
	currFunc->entry = (blocks["entry"] = new CGNode(currFunc, "entry"));
}

void def_block_head(const std::string & blockName)
{
	CGNode *& blockHead = blocks.insert(std::pair<std::string,CGNode* >(blockName, 0)).first->second;
	if (! blockHead) blockHead = new CGNode(currFunc, (blockName).c_str());
	std::cout << "Blockhead " << currFunc << ":" << blockName << "!\n" ;
	currNode = blockHead;
}

NodeTracker(std::map<std::string,FuncInfo> &_functions
	) : functions(_functions), currNode(0), currFunc(0) {}

void  edgeToLabel(const std::string &label) {
	const char* bname = label.c_str();
	bname += strspn(bname, "%@");
	CGNode* &to = blocks.insert(std::pair<std::string,CGNode* > (bname, 0)).first->second;
	if (!to) to = new CGNode(currFunc, bname);
	addEdge(currNode, to);
}

void  unwindToLabel(const std::string &label) {
	const char* bname = label.c_str();
	bname += strspn(bname, "%@");
	CGNode* &to = blocks.insert(std::pair<std::string,CGNode* > (bname, 0)).first->second;
	if (!to) to = new CGNode(currFunc, bname);
	currNode -> unwind = to;
	// //We also add a regular edge in parallel, to enable an easy traversal of everything
	//  addEdge(currNode, to);
}

void set_throws(const std::string &ex) {
	currNode -> throws = ex;
}

void setLandingLabel(const std::string &label, const std::string &ex) {
	const char* bname = label.c_str();
	bname += strspn(bname, "%@");
	CGNode* &to = blocks.insert(std::pair<std::string,CGNode* > (bname, 0)).first->second;
	if (!to) to = new CGNode(currFunc, bname);
	to->catches = ex;
}

void addReturn() {
	currNode->setReturn();
}

void addCall(LLAssign *a) {
	currNode->calls=a;
}

void chainCurrNode()
{
	CGNode* n = new CGNode(currFunc, (currNode -> name+"'").c_str());
	addEdge(currNode, n);
	currNode = n;
}
};

struct FileDomain {
	std::map<std::string, FuncInfo> functions;
};
std::map<std::string,FileDomain> fileDomains;

struct M {
	std::string x, y, c;
	unsigned int flags;
	NodeTracker* track;
	M(NodeTracker* _track) :flags(0U), track(_track) {}
};
typedef LexPattern<M> Patt;

struct LLPattern {
		std::vector<Patt::Elem*> elems;
		void (*handler)(M& m);
		LLPattern(std::initializer_list<Patt::Elem*> _elems, void (*_handler)(M& m))
			:elems(_elems), handler(_handler) {}
	};


static Patt::Var
	xvar("%@", &M::x),
	cvar("%@", &M::c),
	lxvar("%", &M::x),
	gxvar("@", &M::x),
	lyvar("%", &M::y);
static Patt::Any any;
static Patt::Name xname(&M::x);
static Patt::Rest yrest(&M::y);
Patt::Literal *q(const char* l) {return new Patt::Literal( l );}

std::vector <std::string> collectVars(const char* s)
{
	M m(0);
	std::vector <std::string> rc;
	while (s) {
		s = xvar.match_advance(&m,s,true);
		rc.push_back(m.x);
	}
	return rc;	
}

void assignSelector(M& m) {std::cerr << "f1 " << m.x << std::endl; }
void assign(M& m) {std::cerr << "assign " << m.x << std::endl;
	static std::vector<Patt::Elem*>
		esCmp{q("icmp"), q("eq"), &any, &lxvar, q(","), &lyvar},
		esSelType{q("call"), &any, q("@llvm.eh.typeid.for"),q("("),&any,&gxvar};

	M rm(m.track);
	if (Patt::match(&rm,m.y.c_str(),esCmp)) {
		auto xi = var.find(rm.x),
			yi = var.find(rm.y);
		if (xi != var.end() && yi != var.end()
		 && yi->second->kind == A_ES_SELECTOR)
			var[m.x] = new LLAssign(A_ES_MATCH, yi->second->s),
				std::cout << "A_ES_MATCH " << m.x << std::endl;
		} else if (Patt::match(&rm,m.y.c_str(),esSelType)) {
				std::cout << "A_ES_SELECTOR " << m.x << std::endl;

			
							var[m.x] = new LLAssign(A_ES_SELECTOR,rm.x);	
	} else {
		std::vector<std::string> vnames(collectVars(m.y.c_str()));
		std::vector<LLAssign*> vass;
		std::transform(vnames.begin(), vnames.end(),
			std::back_inserter(vass), substituteIfPossible);
		var[m.x] = new LLAssign(vass);
	}
}

void br(M& m) {std::cerr << "f3 " << m.c << " " << m.x << std::endl;
	if (!m.c.empty()) {
		auto xi = var.find(m.c);
		if (xi != var.end()) {
			if (xi->second->kind == A_ES_MATCH) {
				m.track->setLandingLabel(m.x, xi->second->s);
			}
		}
	}
	m.track->edgeToLabel(m.x);
	if (!m.y.empty()) m.track->edgeToLabel(m.y);
}

void unwind(M& m) {std::cerr << "f3 " << m.c << " " << m.x << std::endl;
	m.track->edgeToLabel  (m.x);
	m.track->unwindToLabel(m.y);
}

void ret (M& m) {std::cerr << "r "<< std::endl;
	m.track->addReturn();
}
	
void call (M& m) {std::cerr << "r "<< std::endl;
	auto xi = var.find(m.x) ;
	if (xi != var.end())
		m.track->addCall(xi->second);
	else
		m.track->addCall(new LLAssign(A_VAR,m.x));
}
	
void registerTypeId(M& m) {
	std::cerr << "f5 " << m.x << std::endl;
}	

static LLPattern LLPatternTable[] = {
	LLPattern({&xvar, q("="), &yrest},
		assign),
	LLPattern({q("br"), &any, &cvar, q(","), q("label"), &lxvar, q(","),
		q("label"), &lyvar},
		br),
	LLPattern({q("br"), q("label"), &lxvar},
		br),
	LLPattern({q("to"), q("label"), &lxvar,
		q("unwind"), q("label"), &lyvar},
		unwind),
	LLPattern({q("ret")},
		ret),
	LLPattern({q("resume")},
		ret),
	LLPattern({q("define"),
		new Patt::Peek(1U, {&any, q("internal")}),
		&any, &gxvar, q("(")},
		[](M& m) {m.track->def_fun_head(m.x,
			(m.flags & 1U) != 0U);}) ,
	LLPattern({&xname, q(":")},
		[](M& m) {m.track->def_block_head(m.x);}),
	LLPattern({&any, q("call"), &any, &xvar, q("(")},
		call),
	LLPattern({&any, q("invoke"), &any, &xvar, q("(")},
		call),
	LLPattern({q("invoke"), &any, q("__cxa_throw"), q("("), &any, &gxvar},
		[](M& m) {m.track->set_throws(m.x);}),	
	LLPattern({q("invoke"), &any, q("__exit"), q("(")},
		[](M& m) {m.track->set_throws("exit");}),	
	LLPattern({q("invoke"), &any, q("__abort"), q("(")},
		[](M& m) {m.track->set_throws("exit");})	
	};

void proc(const char* filename) {
	std::string l;
	std::ifstream is(filename);
	NodeTracker track(fileDomains[filename].functions);

	while (!is.eof())
	{
		M m(&track);
		std::getline(is,l);
		for (auto patt: LLPatternTable)
			if (Patt::match(&m,l.c_str(),patt.elems)) patt.handler(m);
	}
}

void qlink(
	std::map<std::string,FileDomain> & fileDomains)
{
	std::map<std::string,FuncInfo*> globalfuncs;
	for (auto &fde:fileDomains)
		for (auto &func:fde.second.functions)
			if (!func.second.internal)
				globalfuncs[func.second.name]=&func.second;
	for (auto &fde:fileDomains)
		for (auto &func:fde.second.functions)
	{
		std::cout<<func.first<<std::endl;
		  dfs_e(func.second.entry , [&] (CGNode* p, CGNode *c) {
			int cnt = 0;
			std::set<std::string> call_vars;
			collectVars(c->calls, call_vars);
			if (!call_vars.empty()) {
				for (const std::string &r: call_vars) {
					FuncInfo * f = 0;
					std::map<std::string, FuncInfo>::iterator i = fde.second.functions.find(r);
					if (i != fde.second.functions.end())
						f = &i->second;
					if (!f) {
						auto gi = globalfuncs.find(r);
						if (gi!=globalfuncs.end())
							f = gi->second;
					}
					if (f) {
						f->invokedFrom.push_back(c);
						c->callsResolved.push_back(f);
						++cnt;
					}
				}
				if (!cnt)
					std::cerr << "No candidate call found for " << 
						c->func->name << ":" << c->name << "("<<c->calls<<"); tried: " << call_vars << std::endl;
			} else if (c->calls) 
					std::cerr << "Internal error: no var collected from calls expression" <<std::endl;
			});
		clear_visit(func.second.entry);
	}
}

inline bool inCone(CGNode* const &what, CGNode* const &dominator)
{
	return what->start_no >  dominator->start_no &&
		what->end_no <= dominator->end_no;
}
inline bool inConeNonStrict(const CGNode* what, const CGNode* dominator)
{
	return what->start_no >= dominator->start_no &&
		what->end_no <= dominator->end_no;
}

template <class C, class Cmp>
inline CGNode *ancestor_lower_bound(CGNode *v, C c, Cmp less) {
	while (!v -> ancestors.empty()&&less(v -> ancestors.back(),c))
	{
			complexity->quasibinary_ancestor_search++;
			v = v -> ancestors.back();
	}
	
	complexity->quasibinary_ancestor_search += v -> ancestors.size()-1;
	for (int i=v -> ancestors.size()-2; i >= 0; --i)
		if (less(v -> ancestors[i],c))
			v = v -> ancestors[i];
	assert (!less(v,c));
	return v;
}

struct BinaryMedianGet {
	int operator () (int lo, int hi) {return (lo + hi)>>1;}
};
#define QUASIBINARY_MARGIN_FACTOR 0.3
struct QuasibinaryMedianGet {
	std::vector<CGNode*> &nodes;
	int operator () (int lo, int hi) {
	int l_margin = ceil(QUASIBINARY_MARGIN_FACTOR*(hi-lo)),
	l_median = (lo + hi)>>1;
	if (lo+l_margin >= hi-l_margin) return l_median;
	CGNode *l = ancestor_lower_bound(nodes[lo+l_margin],
		std::pair<int,int>(lo+l_margin,hi-l_margin),
		[](const CGNode* n, std::pair<int,int>& y){
			return n->start_no > y.first || n->end_no < y.second;});
	return (*std::max_element (l->outgoing.begin(), l->outgoing.end(),
			[l_median](CGNode *x,CGNode *y){
				return std::abs(x->start_no-l_median)<std::abs(y->start_no-l_median);}
		))->start_no;
	}
	QuasibinaryMedianGet(std::vector<CGNode*> &_nodes):nodes(_nodes) {}
};

template<class M>
CGNode *buildBinaryTree(int start, int end, std::vector<CGNode *> &leaf_nodes, M &median_get
){
	CGNode *node = new CGNode(leaf_nodes[0]->func, "%bin"+std::to_string(start)+"-"+std::to_string(end));
	complexity->binary_size++;
	node->start_no = start;
	node->end_no = end;
	if (end <= start+1) {
		//A leaf node: copy outgoings from a "real" node
		node->outgoing = leaf_nodes[start]->outgoing;
	} else {
		int mid = median_get(start, end);
		node->outgoing.push_back(buildBinaryTree(start,mid,leaf_nodes,median_get));
		node->outgoing.push_back(buildBinaryTree(mid,end,leaf_nodes,median_get));
	}	
	return node;
}

void sort_nodes(std::map<std::string,FileDomain> & fileDomains)
{
		for (auto &fde:fileDomains)
		for (auto &func:fde.second.functions)
		{
			int j=0;
			sort_nodes(func.second.entry, j, &func.second.nodes);
			clear_visit(func.second.entry);
		}
}

void setBinHintsR(CGNode *bin, CGNode* vertex, CGNode *CGNode::* hint_field)
{
	if (!vertex->visited) return;
	bool cont = true;
	while ( cont ) {
		cont = false;
		for (auto bo: bin->outgoing)
			if (inConeNonStrict(vertex, bo)) {bin = bo; cont = true; break;}
	}
	vertex ->*hint_field = bin;
	for (auto vo : vertex->outgoing)
		setBinHintsR(vo,bin,hint_field);
}

void setBinHints(CGNode *bin_tree, CGNode* start, CGNode *CGNode::* hint_field) {
	setBinHintsR(bin_tree, start, hint_field);
	clear_visit(start);
}



void buildAuxiliaryTrees()
{
		for (auto &fde:fileDomains)
		for (auto &func:fde.second.functions)
		{
				BinaryMedianGet bmg;
			func.second.binaryTree = 
				buildBinaryTree(0,func.second.nodes.size(),func.second.nodes, bmg);
			QuasibinaryMedianGet qbmg(func.second.nodes);
			setBinHints(func.second.binaryTree, func.second.entry, &CGNode::binary_hint);
			func.second.quasibinaryTree = 
				buildBinaryTree(0,func.second.nodes.size(),func.second.nodes, qbmg);
			setBinHints(func.second.quasibinaryTree, func.second.entry, &CGNode::quasibinary_hint);
	}
}

CGNode *selectStackDown(FuncInfo* f)
{
	if (int c = f->invokedFrom.size())
		return f->invokedFrom[rand()%c];
	return 0;
}

struct DownStream {
	struct Exception;
	typedef std::map<std::string,struct Exception>
		ExMap;
	struct ExOwner {
		ExMap::iterator e;
		ExMap& m;
		ExOwner(ExMap::iterator _e, ExMap& _m) : e(_e), m(_m) {}
		PM get_fill() {return e->second.fill;}
		void consume() {
			complexity->exception_consume += log2s(m.size());
			m.erase(e);
		}
		~ExOwner() {consume();}
	};
	struct Exception {
		PM fill;
		std::vector<std::shared_ptr<ExOwner>> children;
		Exception(PM _fill) :fill(_fill) {}
		get_fill() {
			PM f = fill;
			for (auto &ef : children) f |= ef->get_fill();
			return f;
		}
	};
	DownStream::ExMap  exceptions;
	std::shared_ptr<ExOwner>  addException(const std::string &ex_name, PM fill) {
		auto r = exceptions.emplace(ex_name,fill);
		complexity->exception_create += log2s(exceptions.size());
		if (r.second) {//new entry
			std::set<std::string> childrenVar;
			auto ai = var.find(ex_name);
			if (ai == var.end())
				std::cerr<<"Error: variable not found for " << ex_name << "exception" << std::endl;
			collectVars(var[ex_name], childrenVar);
			for (const std::string &s : childrenVar)
				r.first->second.children.push_back(
				 addException(s, fill));
	}
	return std::shared_ptr<ExOwner>(
		new ExOwner(r.first,exceptions));
}
};


//Using Tarjan algorithm
//Reusing existing node markup:
// 1.  w.start_no > v.start_no  as a "visited" flag
// (as we follow the same dfs order as we used while producing these numbers)
// 2.  v.start_no as an SCC incremental index
// 3.  for a visited node, scc==0 is equivalent to its presence in s stack.

void  assignSCC(CGNode *v, std::vector<CGNode*> &s) {
	s.push_back(v);
    v->scc = 0;
    v->scc_low = v->start_no;

    // Consider successors of v
    for (size_t i = 0U; i < v->outgoing.size(); ++i)
    if (regularEdge(v,i))  {
	  CGNode *w = v->outgoing[i];
      if (w->start_no > v->start_no) {
        // Successor w has not yet been visited; recurse on it
        assignSCC(w, s);
        v->scc_low  = std::min(v->scc_low, w->scc_low);
      } else if (!w->scc) //(w.onStack)
        // Successor w is in stack S and hence in the current SCC
        // If w is not on stack, then (v, w) is a cross-edge in the DFS tree and must be ignored
        // Note: The next line may look odd - but is correct.
        // It says w.index not w.lowlink; that is deliberate and from the original paper
        v->scc_low  = std::min(v->scc_low, w->start_no);

        // If v is a root node, pop the stack and generate an SCC
        if (v->scc_low == v->start_no) {
        //start a new strongly connected component
			CGSCC* scc = new CGSCC(v->func);
		
        {
	        CGNode* w;
	        do
        {
	      w = s.back();
          s.pop_back();
          //add w to current strongly connected component
          scc -> nodes.push_back(w);
          w -> scc = scc; //Also flags 'not on stack'
	    } while (w != v)     ;
		//output the current strongly connected component
	  }
	}
	}
}

void requireSCC(FuncInfo *func) {
	if (!func->entry->scc) {
		std::vector<CGNode*> s;
		assignSCC(func->entry, s);
		//complexity<<mkSCCComplexity();
	}
}

void mkY(CGSCC* start_scc) {
	if (start_scc->y_ready) return;
	start_scc->y_ready = true;
	start_scc->y = 0UL;
	for (auto c: start_scc->nodes) {
		for (auto e: c->outgoing) {
			mkY(e->scc);
			start_scc->y |= e->scc->y;
		}
	}
	//complexity << YComplexity();
}		
void mkY(CGNode* n) {
	requireSCC(n->func);
	mkY(n->func->entry->scc);
}

void invalidate(CGNode* n) {
	requireSCC(n->func);
	//n->func->maxValidSCC = n->scc->no;
}

void revalidate(CGNode* n) {
	requireSCC(n->func);
	//complexity << RevalidateComplexity(n->func->css_list.size() - inval_css);
		//we don't 
}

template <typename T> inline int half_cut(const std::vector<T>& a, T y,
	bool (*less)(T const &, T const &)) {
	int low=0, high=a.size();
	while (high > low) {
	int i = (high + low) >> 1;
	if (a[i] < y)
		high = i;
	else
		low = i+1;
}
	return low;	
}

// inter-func:
//throw -> if catched inside - goes inside the cone
//else goes to terminate...

inline bool leaf(const CGNode *node) {return node->start_no + 1 >= node->end_no;}
int binWhichChild(const CGNode *node, int child_no)
{return node->start_no >= node->outgoing[0]->end_no;}

template <typename A>
void simulateCatch(DownStream &ds, CGNode *catch_node,
	A action)
{
	while (catch_node) {
		if (!catch_node -> catches.empty())
		{
		 auto dei = ds.exceptions.find(catch_node->catches);
		 if (dei != ds.exceptions.end())
		 {
			action(catch_node,dei->second.get_fill());
			ds.exceptions.erase(dei);
		 }
	 	}
		if (catch_node->outgoing.size() > 1 && catch_node -> catches.empty())
		{
				std::cerr<<"Error: Unexpected fork at noncatching CFG fragment in exception handling code"
			<<" at "<<catch_node->func->name<<":"<<catch_node->name<<std::endl;
				catch_node = catch_node->outgoing.back();
		} else if (catch_node->outgoing. size()< 1)
				catch_node = 0;
		else catch_node = catch_node->outgoing.back();
	}
}


template <class Set> struct BlackoutGetter {};

template <> struct BlackoutGetter< Ranges<int> > {
	const std::map<int,int> &r;
	std::map<int,int>::const_iterator it;
	void next() {it++;} 
	bool empty_lower(int bound) {return it == r.end() || white_next()>=bound;}
	size_t size() {return r.size();}
	int white_end() {return it->second;}
	int white_next() {return it->first;}
	BlackoutGetter(Ranges<int> & _ranges, CGNode * _lb) : 
		r(_ranges.getMap()), it(r.lower_bound(_lb->start_no)) {}
};

template <> struct BlackoutGetter < std::set<CGNode *, CGNode::LessStartNo> > {
	std::set<CGNode *, CGNode::LessStartNo> &s;
	std::set<CGNode *, CGNode::LessStartNo>::iterator it;
	void next() {it++;} 
	bool empty_lower(int bound) {return it == s.end() || white_next()>=bound;}
	size_t size() {return s.size();}
	int white_end() {return (*it)->start_no;}
	int white_next() {return (*it)->end_no;}
	BlackoutGetter(std::set<CGNode *, CGNode::LessStartNo> & _s, CGNode * _lb) : 
		s(_s), it(_s.lower_bound(_lb)) {}
};

void propagateBinaryX(CGNode* binaryTree, int no, const TargetVector &x) {
	CGNode *l = binaryTree;
	while (!leaf(l)) {
		l->x |= x;
		int lr = binWhichChild(l,no);
		l = l->outgoing[lr];
		complexity->binary_data_propagate++;
	}
}

void propagateConesX(CGNode *node, const TargetVector &x) {
	while (node) {
		complexity->cone_data_propagate++;
		node->x |=x;
		node = node->ancestors.empty()? 0: node->ancestors.front();
	}
}

class Propagator {
	struct StackFrame {
		Queue minusQueue, plusQueue;
		std::set<CGNode *,CGNode::LessStartNo> plusGuard;
		Ranges<int> minus, plus;
		StackFrame()  {}
	};
	std::unordered_map<FuncInfo*,StackFrame> stack_frames;

inline void addCone(const ModelMode& mode,
	CGNode* vertex, Queue &queue, TerminalVector &y, TargetVector &x)
{
	//Use cone outgoing edges
	std::vector<std::vector<CGNode*> > co(8); //Compute the size properly
#	define COPY_O(t,field) co.push_back(std::vector<CGNode*>()); \
		vertex -> coneOutgoing##field.view.copy_lower( vertex, \
			std::inserter(co.back(),co.back().end())); \
		complexity->heap_edges##field += co.back().size();
	FOR_HEAP_TYPES(,COPY_O)
#	undef COPY_O
	complexity->cones_considered++;
	for (auto n : co.front())
	 	queue.push( n,mode.queue_mode );
	x |= vertex->cone_x;
	y |= vertex->cone_y;
}

void addBinaryOutgoing(const ModelMode &mode, CGNode* binaryTree, int start, int end, Queue &queue,
  TerminalVector &y, TargetVector &x) {
	CGNode *l = binaryTree, *u = binaryTree;
	while (!leaf(l) && l==u) {
		l = l->outgoing[binWhichChild(l,start)];
		u = u->outgoing[binWhichChild(u,end)];
	}
	while (!leaf(l)) {
		int lr = binWhichChild(l,start);
		if (!lr)
			addCone(mode, l->outgoing[1], queue, y, x);
		l = l->outgoing[lr];
		complexity->binary_traversal++;
	}
	while (!leaf(u)) {
		int ur = binWhichChild(u,end);
		if (ur)
			addCone(mode, u->outgoing[0], queue, y, x);;
		u = u->outgoing[ur];
		complexity->binary_traversal++;
	}
}	 

inline void getQHIntervalTargets(CGNode *vertex_hint, const ModelMode &mode,
	int start_no, int end_no,
	Queue &queue, unsigned long y, unsigned long x)
{
		if (mode.quasibinary_outgoing_available())
		{
			addBinaryOutgoing(mode, vertex_hint->quasibinary_hint,
				start_no, end_no, queue, y, x);
		} else if (mode.binary_outgoing_available())
		{
			//Use bin outgoing edges
			addBinaryOutgoing(mode, vertex_hint->binary_hint,
				start_no, end_no, queue, y, x);
		} else //Do a mere DFS descend
		for (auto n : vertex_hint->outgoing)
		 //if (exclude != n)
	 		queue.push(n,mode.queue_mode);//Consider 2 cases
	;
}

template <class Set1>
void getQHTargets(const ModelMode &mode,
	CGNode * vertex ,
	Ranges<int> excludeSet0,
	Set1 excludeSet1,
	Queue &queue, unsigned long &y, unsigned long &x)
{
	bool done_0, done_1;
	int j = 0;
	int istart = vertex->start_no, inext = istart;
	BlackoutGetter<Ranges<int> >	exclude0(excludeSet0,vertex);
	BlackoutGetter<Set1>			exclude1(excludeSet1,vertex);
	complexity->blackout_search+=log2s(exclude0.size())+log2s(exclude0.size());
	while (!((done_0 = exclude0.empty_lower(vertex->end_no))) ||
	   !((done_1= exclude1.empty_lower(vertex->end_no)))
	   )
	{
		j++;
		int iend;
		if ( done_0 || (!done_1 && exclude0.white_end() < exclude1.white_end() )  ) {
				inext = std::max(exclude0.white_next(), inext);
				iend = exclude0.white_end();
				exclude0.next();
			} else {
				inext = std::max(exclude1.white_next(), inext);
				iend = exclude1.white_end();
				exclude1.next();
			}
			if (iend > istart) 
				complexity->intervals_considered++;
				getQHIntervalTargets(vertex, mode, istart, iend,
				queue, y, x); 
			istart = inext;
	}
	if (!j)
		//vertex's cone doesn't contain blackout nodes
		addCone(mode,vertex,queue,y,x);
	else if (istart < vertex->end_no)
		getQHIntervalTargets(vertex, mode, istart, vertex->end_no,
				queue, y, x); 
}
		
	
template <class BlackoutGetter1>
void propagateAndCollect(const ModelMode& mode,
	Queue& queue, Ranges<int> &coveredNodes, BlackoutGetter1 exclude1,
	unsigned long &y, unsigned long &x) {
	while(CGNode* n = queue.remove(mode.queue_mode)) {
		complexity->enqueued2propagate++;
		getQHTargets(mode, n, coveredNodes, exclude1, queue, y, x);
		coveredNodes.insert(n->start_no, n->end_no);
	}
		//newEpoch();
}

void downFrameInFlow(DownStream &ds, FuncInfo *func, unsigned long yPlus, unsigned long yMinus)
{
	for (auto &ex_info : func->throws)
		if (PM fill = ((yPlus<<ex_info.second)&1) | (((yMinus<<ex_info.second)&1)<<1))
			ds.addException(ex_info.first, fill);
}
		
public:
unsigned long modelKsiInStack(const ModelMode &mode, FuncInfo *func,
	const std::vector<CGNode*> &plusNodes, const std::vector<CGNode*> &minusNodes)
{
	if (plusNodes.empty()) return 0UL;
	complexity ->  modeling_runs++;
	unsigned long yPlus = 0UL, yMinus = 0UL, x = 0UL, dummy = 0UL;
	if (!func) func = plusNodes.front()->func;
	for (CGNode *mn:minusNodes) 
		stack_frames[mn->func].minusQueue.push(mn,mode.queue_mode);
	for (CGNode *pn:plusNodes ) {
		StackFrame &f = stack_frames[pn->func];
		f.plusQueue.push(pn,mode.queue_mode);
		f.plusGuard.insert(pn);
	}
	DownStream  ds;
	do {
	StackFrame &frame = stack_frames[func];
	complexity ->  modeling_frames++;
	yPlus = yMinus = 0;
	propagateAndCollect(mode, frame.minusQueue,
		frame.minus, frame.plusGuard,
		yMinus, dummy);
	propagateAndCollect(mode, frame.plusQueue,
		frame.plus, frame.minus,
		yPlus, x);
	//frame.minusQueue.clear(); frame.plusQueue.clear();	

	if (yPlus & ~yMinus)
	if (CGNode *ret = selectStackDown(func)) 
	{
		downFrameInFlow(ds, func, yPlus, yMinus);
		yPlus = yMinus = 0UL;

		//Fall to an outer function here
		frame=stack_frames[func = ret->func];
		
		simulateCatch(ds, ret->unwind, [&frame,&mode](CGNode *r, PM f){
			if (f&2U) frame.minusQueue.push(r,mode.queue_mode);
			else if (f&1U) frame.plusQueue.push(r,mode.queue_mode);
			});
		if (yMinus&1UL) frame.minusQueue.push(ret,mode.queue_mode);
		else if (yPlus&1UL) frame.plusQueue.push(ret,mode.queue_mode);		
		}
	} while (yPlus & ~yMinus);
	return x;
}
}; //Propagator

void applyPruneDuplicateEntryFires(CGNode* node) //move to CGNode?
{
					//Asscend
#				define PRUNE_O(t,field)\
				while (!node->field.fires.empty()) \
				{\
					node->field.view = HEAP_TYPE(t)::erase_from_last_recent(node->field.fires.back()); \
					node->field.fires.pop_back(); \
					std::cout << "Pruning " << node << std::endl; \
				}
				FOR_HEAP_TYPES(coneOutgoing,PRUNE_O)
#				undef PRUNE_O
}

void addForeignOutgoing(CGNode *node, CGNode *o, std::vector<CGNode*> &depthUpNodes)
{
				if (o->lastSeenOugoingFrom) {
					int ir = half_cut<CGNode *>(depthUpNodes,o->lastSeenOugoingFrom,&inCone);
					CGNode *redundantAt = ir > 0 ? depthUpNodes[ir-1]:0;

					if (redundantAt)
					{
						assert(inConeNonStrict(o->lastSeenOugoingFrom, redundantAt));
						assert(inConeNonStrict(node, redundantAt));
#						ifndef NDEBUG
						for (auto ro: redundantAt->outgoing)
							assert(!inConeNonStrict(o->lastSeenOugoingFrom, ro) ||
							(!inConeNonStrict(node, ro)));
#						endif							
#						define ADD_FIRE_O(t,field) redundantAt->field . fires.push_back(o->field.handle);
						FOR_HEAP_TYPES(coneOutgoing,ADD_FIRE_O)
#						undef ADD_FIRE_O
					}
				}
				o->lastSeenOugoingFrom = node;
				applyPruneDuplicateEntryFires(node);
#				define INSERT_O(t,field) o->field.handle = node->field.view.insert(o);
				FOR_HEAP_TYPES(coneOutgoing,INSERT_O)
#				undef INSERT_O
				
	}

void findStaticThrows() {
	std::vector< std::pair<CGNode *,std::string> > queue;
	for (auto &fd: fileDomains)
	for (auto &f: fd.second.functions)
	for (auto n: f.second.nodes) if (!n->throws.empty())
			queue.push_back(std::pair<CGNode *,std::string>(n,n->throws));
	while (!queue.empty()) {
		std::pair<CGNode *,std::string> &to_add = queue.back();
		queue.pop_back(); //optimiza not to copy the string?
		DownStream e;
		e.addException(to_add.second, 0U);
		simulateCatch(e,  to_add.first,[&to_add](CGNode* t, PM){to_add.first->outgoing.push_back(t);});
		if (!e.exceptions.empty() //Uncaught
			&& to_add.first->func->throws.insert(std::pair<std::string,int>(to_add.second,to_add.first->func->throws.size()+1/* #0 is reserved? */)).second)
			for (auto c: to_add.first->func->invokedFrom)
					queue.push_back(std::pair<CGNode *,std::string>(c, to_add.second));
	}
}

void buildConeOutgoingR(CGNode* node, std::vector<CGNode*> &depthUpNodes, int depth )
{
	node->visited = true;
	complexity->depth_sum++;
	depthUpNodes[depth] = node;
		for (auto o: node->outgoing)
		if (!inCone(o, node))
			addForeignOutgoing(node,o,depthUpNodes);
		else if (!o->visited) {
			o->lastSeenOugoingFrom = 0;
			assert(inCone(o, node));
			buildConeOutgoingR(o, depthUpNodes, depth + 1);
			applyPruneDuplicateEntryFires(node);
#				define MERGE_O(t,field)	node->field.view |= o->field.view ;
				FOR_HEAP_TYPES(coneOutgoing,MERGE_O)
#				undef MERGE_O
		} 
}

void buildConeOutgoing(CGNode * start) {
	std::vector<CGNode*> depthUpNodes(start->end_no, 0);
	buildConeOutgoingR(start, depthUpNodes, 0);
	clear_visit(start);
}

void setAncestorsR(CGNode* node, std::vector<CGNode*> &depthUpNodes, int depth )
{
	node->visited = true;
	depthUpNodes[depth] = node;
		for (auto o: node->outgoing)
		if (!o->visited) {
			assert(inCone(o, node));
			int depthplus = depth + 1, p, i = 0;
			do {
					o->ancestors.push_back(depthUpNodes[p=depthplus-(1<<i)]);
					complexity->ancestor_net++;
			}
			while ((1<<i++) & ~depthplus);
			setAncestorsR(o, depthUpNodes, depthplus);
		}
}

void setAncestors(CGNode *start)
{
	std::vector<CGNode*> depthUpNodes(start->end_no, 0);
	setAncestorsR(start, depthUpNodes, 0);
	clear_visit(start);
}

void setAncestors(std::map<std::string,FileDomain> & fileDomains)
{
		for (auto &fde:fileDomains)
		for (auto &func:fde.second.functions)
			setAncestors(func.second.entry);
}

void DDDistributeR(CGNode* n, const TargetVector& x) {
	if (!n->visited) return;
	FuncInfo *fn = n->func;
	complexity->model_dd_invocation_fanout_sum++;
	for ( auto &c: n->func->invokedFrom )
//		switch(f->invokeAccountingTtype) {
//		case INLINED: //? Not yet supported 
// 			break;
//		case CALL:
			// Need to update using Y iface:
		DDDistributeR(c,x);
//			break;
//	}		
	if (fn->binaryTree)
			propagateBinaryX(fn->binaryTree,n->start_no,x);
	if (fn->quasibinaryTree)
			propagateBinaryX(fn->quasibinaryTree,n->start_no,x);
	if (fn->entry)
			propagateConesX(n, x);	
}

void modelKsiAll()
{
	Propagator propagator;
	//Iterate through all
	for (const auto &mode: ModelMode::allModes())
	{
		complexity = &modeComplexity[mode.getDesc()];
	for (auto fdi: fileDomains)
	  for (auto fi: fdi.second.functions) {
		for (auto ni: fi.second.nodes) {
			std::set<CGNode*> ys;
			for (auto yi: ni ->outgoing) {
				if (ys.find(yi) == ys.end()) {
					propagator.modelKsiInStack(mode, &fi.second,
					std::vector<CGNode *>{ni}, std::vector<CGNode *>{yi});
					ys.insert(yi);
				}
			}
		}
	}
	complexity = &commonComplexity;
}
}

void modelDDDistributeAll()
{
	//Iterate through all
	for (auto &fdi: fileDomains)
	  for (auto &f: fdi.second.functions)
		for (auto node: f.second.nodes)
			DDDistributeR(node,0UL);
}

std::ostream &reportHeapComplexity(std::ostream &os)
{
#	define REPORT_COMPLEXITY(t,f) \
	for (int i=0; i<heap_stat::EVENT_KIND_COUNT; ++i) os<<#t<<" #"<<i<<": "<<CountHeapEvents<t>::counts[i] << std::endl; 
	FOR_HEAP_TYPES(coneOutgoing,REPORT_COMPLEXITY)
#	undef REPORT_COMPLEXITY
	return os;
}

int main (int argc, char** argv) {
	if (argc>1) {
		for (int n=1; n<argc; ++n) proc(argv[n]);
		qlink(fileDomains);
		findStaticThrows();
		sort_nodes(fileDomains);
		setAncestors(fileDomains);
		buildAuxiliaryTrees();
		for (auto &fd: fileDomains) {
			std::cout << "=========  File: "<<fd.first << " =========\n" ;
			complexity->functions+=fd.second.functions.size();
			for (auto &f: fd.second.functions) {
				//Build cone outgoing heaps
				complexity->nodes+=f.second.nodes.size();
				if (!f.second.invokedFrom.empty()) {
					complexity->invoked++;
					complexity->invocations+=f.second.invokedFrom.size();
				}
				buildConeOutgoing(f.second.entry);
				buildConeOutgoing(f.second.binaryTree);
				buildConeOutgoing(f.second.quasibinaryTree);
				std::cout << f.second;
			}
		}
	}
	else {
		std::cerr << "ll filename argument needed" << std::endl;
		return 2;
	}
	modelKsiAll();
	modelDDDistributeAll();
	printStatistics(std::cout);
	reportHeapComplexity(std::cout);
	return 0;
}

