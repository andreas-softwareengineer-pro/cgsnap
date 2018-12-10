#include  <map>
#include  "cgnode.h"

#define stat_field(c,x) int x;
#define stat_field_init(c,x) x = 0;
#define init_stat_field_string(c,x) stat_field[#x]=&ComplexityStat::x;
#define stat_field_initiaizer(c,x) {#x, &ComplexityStat::x},

//Count leading zeros
int clz(uint32_t x) {
  if (x == 0)
	return 32;
  int n = 0;
  if ((x & 0xFFFF0000) == 0)
	n = n + 16, x = x << 16;
  if ((x & 0xFF000000) == 0)
	n = n + 8, x = x << 8;
  if ((x & 0xF0000000) == 0)
	n = n + 4, x = x << 4;
  if ((x & 0xC0000000) == 0)
	n = n + 2, x = x << 2;
  if ((x & 0x80000000) == 0)
	n = n + 1;
  return n;
}

int log2s(uint32_t x) {
	return x? 32-clz(x) : 0; //Update to accout for half-2-powers?
}

#define STATISTIC_FIELDS(q) \
	q(,nodes)\
	q(,functions)\
	q(,invoked)\
	q(,invocations)\
/* static exception modeling */ \
	q(,exception_consume)\
	q(,exception_create)\
/* building trees */ \
	q(,binary_size)\
	q(,quasibinary_ancestor_search)\
	q(,ancestor_net)\
/* propagating q(,info) */ \
	q(,depth_sum)\
	q(,model_dd_invocation_fanout_sum)\
	q(,model_dd)\
	q(,cone_data_propagate)\
	q(,binary_data_propagate)\
/* modeling */ \
	q(,modeling_runs)\
	q(,modeling_frames)\
/* propagation q(,cost) */ \
	q(,blackout_search)\
	q(,intervals_considered)\
	q(,cones_considered)\
	q(,binary_traversal)\
	q(,enqueued2propagate)\
	FOR_HEAP_TYPES(heap_edges,q)

struct ComplexityStat {
	STATISTIC_FIELDS(stat_field);
};

ComplexityStat commonComplexity;
static std::map<std::string , ComplexityStat> modeComplexity;
thread_local ComplexityStat * complexity = &commonComplexity;

typedef int ComplexityStat::* CP;
static std::map<const char *,CP > complexityStatChart =
	{STATISTIC_FIELDS(stat_field_initiaizer)};
	
std::ostream &printStat(std::ostream &os, ComplexityStat& complexity, const std::string &prefix) {
	for (auto &c: complexityStatChart)
	if (complexity.*c.second)
		os << prefix << " " << c.first << ": " << complexity.*c.second << std::endl;
	return os; 
}

std::ostream &printStatistics(std::ostream &os) {
	printStat(os, commonComplexity, "common");
	for (auto &m: modeComplexity)
		printStat(os, m.second, m.first);
	return os;
}
	

