# include <iostream>
# include <map>
# include <stdlib.h>
# include <string.h>
# include <unordered_map>
# include <assert.h>
# include "mark_interest.h"

//Module test

	std::pair<std::string,std::string> edgeSpec [] = {
		std::pair<std::string,std::string>("t0","a1"),
		std::pair<std::string,std::string>("a1","a2"),
		std::pair<std::string,std::string>("t1","a3"),
		std::pair<std::string,std::string>("a2","a3"),
		std::pair<std::string,std::string>("t2","a4"),
		std::pair<std::string,std::string>("t3","a3"),
		std::pair<std::string,std::string>("a3","a4"),
		std::pair<std::string,std::string>("a4","a5"),
		std::pair<std::string,std::string>("a5","a6"),
		std::pair<std::string,std::string>("a4","a6"),
		std::pair<std::string,std::string>("t4","a5"),
		std::pair<std::string,std::string>("a6","a7"),
		std::pair<std::string,std::string>("t5","a7"),
		std::pair<std::string,std::string>("a7","a8"),
		std::pair<std::string,std::string>("a9","a8"),
		std::pair<std::string,std::string>("a8","a3")
	};
	
	struct TestNode {
		TestNode *next;
		typedef MarkInterest<TestNode>::Edge  Edge;
		Linked<Edge,&Edge::next> outgoing,boundary;
		TestNode *owner;
		bool interesting;
		TestNode(bool isRoot) :owner(isRoot? this : 0), interesting(isRoot) {}
		bool ownerLive() {return owner && owner->interesting;}
	};
	
		std::map<std::string,TestNode> objs;
	
		void print() {
		for (auto &se: objs)
			std::cout << se.first << " [" << &se.second << "] "
					 << se.second.owner << " " << se.second.interesting << std::endl;
	}
	
	MarkInterest<TestNode> mi;
	void buildGraph() {
		for (std::pair<std::string,std::string> s:edgeSpec)
			{
				TestNode &src = objs.emplace(s.first, s.first[0]=='t').first->second,
						&dst = objs.emplace(s.second, s.second[0]=='t').first->second;
				mi.insert(&src,&dst);
			}
		}

	int checkTotal() {
		int sz = 0;
		sz += mi.queue.countsize();
		sz += mi.erase_queue.countsize();
		for(auto &se: objs) {
			sz += se.second.outgoing.countsize();
			sz += se.second.boundary.countsize();
		}
		return sz;
	}

	int main(int argc, char **argv) {
		buildGraph();
		int last_total = checkTotal();

			while (true) {
			char buff[1024], *s = fgets(buff, 1024, stdin);
			//trim
			while (isspace(s[0])) s++;
			char cmd = *s++;
			while (isspace(s[0])) s++;
			if (!cmd) continue;
			int l = strlen(s);
			while (isspace(s[l-1])) s[--l] = 0;
			switch (cmd) {
				case 'a':
				{
						auto oi = objs.find(s);
					if (oi != objs.end())
						{
							std::cerr<<"ok"<<std::endl;
							oi->second.interesting = (true);
							mi.enqueue(&oi->second);
						}
				}
					break;
				case 'p':
					mi.markInterest(atoi(s));
					break;
				case 'd':
				{
					auto oi = objs.find(s);
					if (oi != objs.end())
						{
							std::cerr<<"ok"<<std::endl;
							oi	->second.interesting = (false);
							mi.enqueue(&oi->second);
						}
				}
				break;
				case 'q':
					exit(atoi(s));
				default:
					std::cerr<<"Unknown command"<<std::endl;
					break;
				}
			print();
			int new_total = checkTotal();
			if (last_total != new_total) {
				std::cerr << "TOTAL CHANGED " << last_total << " " << new_total << std::endl;
				last_total = new_total;
			}
				std::cerr << "total " << last_total <<  std::endl;
			}
		}
	