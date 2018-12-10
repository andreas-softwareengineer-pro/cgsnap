  template <class T> class Ranges {
	  std::map<T,T> r; //range end -> range begin
	public:
	  bool contains (const T& t) const {
		  typename std::map<T,T>::const_iterator i = r.lower_bound(t);
		  return (i != r.end()) ? i->second <= t : false; 
	  }

	  Ranges& insert(T start, T end) {
		  if (start > end) std::swap(start,end);
		  typename std::map<T,T>::iterator i = r.lower_bound(start);
		  //erase & update begin..
		  while (i!=r.end() && i->second <=end) {
			  if (start > i->second) start = i->second;
			  if (i->first >= end) end = i++->first;
			  else r.erase(i++);
		  }

		  if (i != r.end() && i->first==end) {  //has [x..end]
		        i->second = start;
	      } else // new range
              r.insert(std::pair<T,T>(end,start));
          return *this;
      }
      
      Ranges& operator |= (const Ranges& y) {
	      typename std::map<T,T>::const_iterator i = y.r.begin();
	      for (; i!= y.r.end(); ++i) insert(i->second,i->first);
	      return * this;
      }
      
      void clear() {r.clear();}

      const std::map<int,int> &getMap() {return r;}

      Ranges<T> operator ~() const {
	      Ranges<T> out;
	      T b = std::numeric_limits<T>::min();
	      typename std::map<T,T>::const_iterator i = r.begin();
	      for (; i!=r.end(); ++i) {
		      if (i->second > b)  out.r[i->second - 1] = b;
		      b = i -> first;
		      if (b >= std::numeric_limits<T>::max()) return out;
		      b++;
	      }
	      out.r[std::numeric_limits<T>::max()] = b;
	      return out;
      }
  };
