template <class M>
struct LexPattern {
struct Elem {
	virtual bool is_any() {return false;}
	virtual const char* match_advance(M* m, const char* s, bool free_begin) = 0;
};

	static bool match(M* m, const char* s, std::vector<Elem*> &patt) {
		int j = 0;
		const char *w0 = 0, *w = s;
		while (j < (int)patt.size()) 
			if (j > 0 && patt[j-1]->is_any()) {
				w0 = w = patt[j]->match_advance(m,w,true);
				if (!w0) return false;
				j++;
			} else {
				w = patt[j]->match_advance(m,w,false);
				if (w) j++;
				else {
					//Try advancing most recent free begin
					while (j > 0 && !patt[j-1]->is_any()) --j;
					if (!j) return false;
					w = w0;
				}
			}
		return true;
}

enum CharKind {CK_SPACE, CK_WORD, CK_PUNCT};
static CharKind ckind(char c) {
	if (!c || isspace(c)) return CK_SPACE;
	if (isalnum(c) || c=='.' || c=='_') return CK_WORD;
	return CK_PUNCT;
}

static bool is_boundary(char c0, char c1) {
	CharKind k0=ckind(c0), k1=ckind(c1);
	return k0 != k1 || k0 == CK_PUNCT;
}

static const char* skip_space(const char* s) {
	while (isspace(*s)) s++;
	return s;
}

struct Literal : public Elem{
	std::string literal;
	virtual const char* match_advance(M* m, const char* s, bool free_begin) {
		int l = literal.length();
		if (free_begin) {
				const char* h = s;
				do h = strstr(h + (h != s),literal.c_str());
				while (h &&
				 (h!=s && !(is_boundary(h[-1],h[0])) //left bound ok
				 && is_boundary(h[l-1],h[l]))
				 ); //right bound ok
				return h? h+l : 0;
		} else {
				s = skip_space(s);
			return 
				(!strncmp(literal.c_str(),s,literal.length())
				&& 	is_boundary(s[l-1],s[l])) ?
			s+l : 0; //right bound ok
		}
	}
	Literal(const char* _literal) : literal(_literal) {}
};

struct Any : public Elem {
	virtual bool is_any() {return true;}
	virtual const char* match_advance(M* m, const char* s, bool free_begin) {
		return s;			
	}
};


struct Var : public Elem {
	std::string start_chars;
	std::string M::* storeName;
	virtual const char* match_advance(M* m, const char* s, bool free_begin) {			
			if (free_begin) s+=strcspn(s,start_chars.c_str());
			else s=skip_space(s);
			if (!*s || !strchr(start_chars.c_str(),*s)) return 0;

			const char* w = s; do w++; while (ckind(*w) == CK_WORD);
			if (storeName) (m->*storeName).assign(s, w-s);
			return w;
}

	Var(const std::string &_start_chars, std::string M::* _storeName) :
		start_chars(_start_chars), storeName(_storeName) {}
};

struct Name : public Elem {
	std::string M::* storeName;
	virtual const char* match_advance(M* m, const char* s, bool free_begin) {			
			const char* h = s;
			if (free_begin) 
				do {
					if (h != s) ++h;
					while(*h && ckind(*h) != CK_WORD)
				++h ;}
					while (h!=s && !is_boundary(h[-1],h[0]));
			else {
				h=skip_space(h);
				if (ckind(*h) != CK_WORD) return 0;
			}

	const char* w = h; do w++; while (ckind(*w) == CK_WORD);
	if (storeName) (m->*storeName).assign(h, w-h);
	return w;
}

	Name( std::string M::* _storeName) : storeName(_storeName) {}
};

struct Rest : public Elem {
	std::string  M::* storeName;
	virtual const char* match_advance(M* m, const char* s, bool) {			
	if (storeName) (m->*storeName).assign(s);
	return s+strlen(s); {
	}
}

	Rest(std::string M::* _storeName) :
		storeName(_storeName) {}
};

struct Peek : public Elem {
	std::vector<Elem*> elems;
	unsigned long flag;
	virtual const char* match_advance(M* m, const char* s, bool) {			
		if (match(m,s,elems)) 
			(m->flags)|=flag;
		return s;  //unchanged
	}
	Peek(unsigned long _flag,
		std::initializer_list<Elem*> _elems)
		: elems(_elems), flag(_flag) {}
};
};

