/* Generated by Nim Compiler v1.9.3 */
#define NIM_INTBITS 32

#include "nimbase.h"
#include <stdio.h>
#include <string.h>
#undef LANGUAGE_C
#undef MIPSEB
#undef MIPSEL
#undef PPC
#undef R3000
#undef R4000
#undef i386
#undef linux
#undef mips
#undef near
#undef far
#undef powerpc
#undef unix
#define nimfr_(proc, file) \
  TFrame FR_; \
  FR_.procname = proc; FR_.filename = file; FR_.line = 0; FR_.len = 0; nimFrame(&FR_);

  #define nimfrs_(proc, file, slots, length) \
    struct {TFrame* prev;NCSTRING procname;NI line;NCSTRING filename;NI len;VarSlot s[slots];} FR_; \
    FR_.procname = proc; FR_.filename = file; FR_.line = 0; FR_.len = length; nimFrame((TFrame*)&FR_);

  #define nimln_(n, file) \
    FR_.line = n; FR_.filename = file;
typedef struct NimStrPayload NimStrPayload;
typedef struct NimStringV2 NimStringV2;
struct NimStrPayload {NI cap;
NIM_CHAR data[SEQ_DECL_SIZE];
};
struct NimStringV2 {NI len;
NimStrPayload* p;
};
typedef NIM_CHAR tyArray__eVNFTutn6un5gcq48fQLdg[65];
N_LIB_PRIVATE N_NIMCALL(NI, writeFloatToBufferSprintf__stdZformatfloat_80)(NIM_CHAR* buf__aghmAHvd9cdW1At39cKpIAdw, NF value__FfZz1mqKLAloLO9az2ZowSQ);
N_LIB_PRIVATE N_NIMCALL(void, writeToBuffer__stdZformatfloat_71)(NIM_CHAR* buf__GlezLOyrOsddrPsVvk9b9bhg, NCSTRING value__jUAdECcqOvFmaMsAxTOuVA);
static N_INLINE(void, nimFrame)(TFrame* s__ZBM8FF7uw5z5eZuYXmXfaw);
N_LIB_PRIVATE N_NOINLINE(void, callDepthLimitReached__system_4495)(void);
static N_INLINE(void, popFrame)(void);
static N_INLINE(NIM_BOOL*, nimErrorFlag)(void);
N_LIB_PRIVATE N_NIMCALL(void, addCstringN__stdZformatfloat_5)(NimStringV2* result__aIuLXCxBFtPuTt3aF8QCUw, NCSTRING buf__BB6c1qB0qG189c0DHAVckSA, NI buflen__b58r4lVY2Ql9aQEX8dEXjkA);
N_LIB_PRIVATE N_NIMCALL(void, setLengthStrV2)(NimStringV2* s__19atMbr42TbRkrQSPFxaPZg, NI newLen__zdmruIpEqwmFKxid9btfbJw);
extern NIM_THREADVAR TFrame* framePtr__system_3928;
extern NIM_THREADVAR TFrame* framePtr__system_3928;
extern NIM_THREADVAR TFrame* framePtr__system_3928;
extern NIM_THREADVAR TFrame* framePtr__system_3928;
extern NIM_THREADVAR TFrame* framePtr__system_3928;
extern NIM_THREADVAR TFrame* framePtr__system_3928;
extern NIM_THREADVAR NIM_BOOL nimInErrorMode__system_4298;

#line 549 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
static N_INLINE(void, nimFrame)(TFrame* s__ZBM8FF7uw5z5eZuYXmXfaw) {
#line 550 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
	{
#line 550 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
		if (!(framePtr__system_3928 == ((TFrame*) NIM_NIL))) goto LA3_;

#line 551 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"

#line 551 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
		(*s__ZBM8FF7uw5z5eZuYXmXfaw).calldepth = ((NI16)0);	}
	goto LA1_;
	LA3_: ;
	{
#line 554 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"

#line 554 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"

#line 554 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
		(*s__ZBM8FF7uw5z5eZuYXmXfaw).calldepth = (NI16)((*framePtr__system_3928).calldepth + ((NI16)1));	}
	LA1_: ;

#line 556 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"

#line 556 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
	(*s__ZBM8FF7uw5z5eZuYXmXfaw).prev = framePtr__system_3928;
#line 557 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"

#line 557 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
	framePtr__system_3928 = s__ZBM8FF7uw5z5eZuYXmXfaw;
#line 558 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
	{
#line 558 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
		if (!((*s__ZBM8FF7uw5z5eZuYXmXfaw).calldepth == ((NI16)2000))) goto LA8_;

#line 558 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"

#line 558 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
		callDepthLimitReached__system_4495();
	}
	LA8_: ;
}

#line 99 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
static N_INLINE(void, popFrame)(void) {
#line 100 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"

#line 100 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
	framePtr__system_3928 = (*framePtr__system_3928).prev;}

#line 41 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
N_LIB_PRIVATE N_NIMCALL(void, writeToBuffer__stdZformatfloat_71)(NIM_CHAR* buf__GlezLOyrOsddrPsVvk9b9bhg, NCSTRING value__jUAdECcqOvFmaMsAxTOuVA) {	
#line 42 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
NI i;	nimfr_("writeToBuffer", "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 42 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	nimln_(42, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");	i = ((NI)0);	{
#line 43 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		nimln_(43, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");		while (1) {
#line 43 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"

#line 43 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
			if (!!(((NU8)(value__jUAdECcqOvFmaMsAxTOuVA[i]) == (NU8)(0)))) goto LA2			;

#line 44 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
			nimln_(44, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 44 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
			buf__GlezLOyrOsddrPsVvk9b9bhg[(i)- 0] = value__jUAdECcqOvFmaMsAxTOuVA[i];
#line 45 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
			nimln_(45, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");			i += ((NI)1);		} LA2: ;
	}
	popFrame();}

#line 423 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
static N_INLINE(NIM_BOOL*, nimErrorFlag)(void) {	
#line 423 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
NIM_BOOL* result;	result = (NIM_BOOL*)0;
#line 424 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"

#line 424 "/home/vincent/nim-1.9.3/nim/lib/system/excpt.nim"
	result = (&nimInErrorMode__system_4298);	return result;}

#line 47 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
N_LIB_PRIVATE N_NIMCALL(NI, writeFloatToBufferSprintf__stdZformatfloat_80)(NIM_CHAR* buf__aghmAHvd9cdW1At39cKpIAdw, NF value__FfZz1mqKLAloLO9az2ZowSQ) {	
#line 47 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
NI result;	
#line 52 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
NI n;	int T1_;	
#line 53 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
NIM_BOOL hasDot;NIM_BOOL* nimErr_;	nimfr_("writeFloatToBufferSprintf", "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");{nimErr_ = nimErrorFlag();	result = (NI)0;
#line 52 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	nimln_(52, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 52 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"

#line 52 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	T1_ = (int)0;	T1_ = sprintf(((NCSTRING) (buf__aghmAHvd9cdW1At39cKpIAdw)), "%.16g", value__FfZz1mqKLAloLO9az2ZowSQ);	n = ((NI) (T1_));
#line 53 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	nimln_(53, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");	hasDot = NIM_FALSE;	{		
#line 54 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
NI i;		
#line 75 "/home/vincent/nim-1.9.3/nim/lib/system/iterators_1.nim"
NI colontmp_;		
#line 90 "/home/vincent/nim-1.9.3/nim/lib/system/iterators_1.nim"
NI res;		i = (NI)0;		colontmp_ = (NI)0;
#line 75 "/home/vincent/nim-1.9.3/nim/lib/system/iterators_1.nim"
		nimln_(75, "/home/vincent/nim-1.9.3/nim/lib/system/iterators_1.nim");
#line 54 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		nimln_(54, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 54 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		colontmp_ = (NI)(n - ((NI)1));
#line 90 "/home/vincent/nim-1.9.3/nim/lib/system/iterators_1.nim"
		nimln_(90, "/home/vincent/nim-1.9.3/nim/lib/system/iterators_1.nim");		res = ((NI)0);		{
#line 91 "/home/vincent/nim-1.9.3/nim/lib/system/iterators_1.nim"
			nimln_(91, "/home/vincent/nim-1.9.3/nim/lib/system/iterators_1.nim");			while (1) {
#line 91 "/home/vincent/nim-1.9.3/nim/lib/system/iterators_1.nim"
				if (!(res <= colontmp_)) goto LA4				;

#line 54 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
				nimln_(54, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 54 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
				i = res;
#line 54 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
				{
#line 55 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
					nimln_(55, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");					if (!((NU8)(buf__aghmAHvd9cdW1At39cKpIAdw[(i)- 0]) == (NU8)(44))) goto LA7_;

#line 56 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
					nimln_(56, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 56 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
					buf__aghmAHvd9cdW1At39cKpIAdw[(i)- 0] = 46;
#line 57 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
					nimln_(57, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 57 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
					hasDot = NIM_TRUE;				}
				goto LA5_;
				LA7_: ;
				{
#line 58 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
					nimln_(58, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");					if (!(((NU8)(buf__aghmAHvd9cdW1At39cKpIAdw[(i)- 0])) >= ((NU8)(97)) && ((NU8)(buf__aghmAHvd9cdW1At39cKpIAdw[(i)- 0])) <= ((NU8)(122)) || ((NU8)(buf__aghmAHvd9cdW1At39cKpIAdw[(i)- 0])) >= ((NU8)(65)) && ((NU8)(buf__aghmAHvd9cdW1At39cKpIAdw[(i)- 0])) <= ((NU8)(90)) || ((NU8)(buf__aghmAHvd9cdW1At39cKpIAdw[(i)- 0])) == ((NU8)(46)))) goto LA10_;

#line 59 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
					nimln_(59, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 59 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
					hasDot = NIM_TRUE;				}
				goto LA5_;
				LA10_: ;
				LA5_: ;

#line 93 "/home/vincent/nim-1.9.3/nim/lib/system/iterators_1.nim"
				nimln_(93, "/home/vincent/nim-1.9.3/nim/lib/system/iterators_1.nim");				res += ((NI)1);			} LA4: ;
		}
	}

#line 60 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	nimln_(60, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");	{
#line 60 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		if (!!(hasDot)) goto LA14_;

#line 61 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		nimln_(61, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 61 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		buf__aghmAHvd9cdW1At39cKpIAdw[(n)- 0] = 46;
#line 62 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		nimln_(62, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 62 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"

#line 62 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		buf__aghmAHvd9cdW1At39cKpIAdw[((NI)(n + ((NI)1)))- 0] = 48;
#line 63 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		nimln_(63, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 63 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"

#line 63 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		buf__aghmAHvd9cdW1At39cKpIAdw[((NI)(n + ((NI)2)))- 0] = 0;
#line 64 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		nimln_(64, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 64 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"

#line 64 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		result = (NI)(n + ((NI)2));	}
	goto LA12_;
	LA14_: ;
	{
#line 66 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		nimln_(66, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 66 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		result = n;	}
	LA12_: ;

#line 70 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	nimln_(70, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");	{
#line 70 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"

#line 70 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		if (!(((NU8)(buf__aghmAHvd9cdW1At39cKpIAdw[((NI)(n - ((NI)1)))- 0])) == ((NU8)(110)) || ((NU8)(buf__aghmAHvd9cdW1At39cKpIAdw[((NI)(n - ((NI)1)))- 0])) == ((NU8)(78)) || ((NU8)(buf__aghmAHvd9cdW1At39cKpIAdw[((NI)(n - ((NI)1)))- 0])) == ((NU8)(68)) || ((NU8)(buf__aghmAHvd9cdW1At39cKpIAdw[((NI)(n - ((NI)1)))- 0])) == ((NU8)(100)) || ((NU8)(buf__aghmAHvd9cdW1At39cKpIAdw[((NI)(n - ((NI)1)))- 0])) == ((NU8)(41)))) goto LA19_;

#line 71 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		nimln_(71, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 71 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		writeToBuffer__stdZformatfloat_71(buf__aghmAHvd9cdW1At39cKpIAdw, "nan");
		if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;
#line 72 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		nimln_(72, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 72 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		result = ((NI)3);	}
	goto LA17_;
	LA19_: ;
	{
#line 73 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		nimln_(73, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 73 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		if (!((NU8)(buf__aghmAHvd9cdW1At39cKpIAdw[((NI)(n - ((NI)1)))- 0]) == (NU8)(70))) goto LA22_;

#line 74 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
		nimln_(74, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");		{
#line 74 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
			if (!((NU8)(buf__aghmAHvd9cdW1At39cKpIAdw[(((NI)0))- 0]) == (NU8)(45))) goto LA26_;

#line 75 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
			nimln_(75, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 75 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
			writeToBuffer__stdZformatfloat_71(buf__aghmAHvd9cdW1At39cKpIAdw, "-inf");
			if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;
#line 76 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
			nimln_(76, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 76 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
			result = ((NI)4);		}
		goto LA24_;
		LA26_: ;
		{
#line 78 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
			nimln_(78, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 78 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
			writeToBuffer__stdZformatfloat_71(buf__aghmAHvd9cdW1At39cKpIAdw, "inf");
			if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;
#line 79 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
			nimln_(79, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 79 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
			result = ((NI)3);		}
		LA24_: ;
	}
	goto LA17_;
	LA22_: ;
	LA17_: ;
	}BeforeRet_: ;
	popFrame();	return result;}

#line 17 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
N_LIB_PRIVATE N_NIMCALL(void, addCstringN__stdZformatfloat_5)(NimStringV2* result__aIuLXCxBFtPuTt3aF8QCUw, NCSTRING buf__BB6c1qB0qG189c0DHAVckSA, NI buflen__b58r4lVY2Ql9aQEX8dEXjkA) {	
#line 19 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
NI oldLen;	
#line 20 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
NI newLen;	void* T1_;	nimfr_("addCstringN", "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 19 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	nimln_(19, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 19 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	oldLen = (*result__aIuLXCxBFtPuTt3aF8QCUw).len;
#line 20 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	nimln_(20, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 20 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	newLen = (NI)(oldLen + buflen__b58r4lVY2Ql9aQEX8dEXjkA);
#line 21 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	nimln_(21, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");	setLengthStrV2((&(*result__aIuLXCxBFtPuTt3aF8QCUw)), ((NI) (newLen)));
#line 22 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	nimln_(22, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 22 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	T1_ = (void*)0;	T1_ = memcpy(((void*) ((&(*result__aIuLXCxBFtPuTt3aF8QCUw).p->data[oldLen]))), ((void*) (buf__BB6c1qB0qG189c0DHAVckSA)), ((size_t) (buflen__b58r4lVY2Ql9aQEX8dEXjkA)));	popFrame();}

#line 95 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
N_LIB_PRIVATE N_NIMCALL(void, addFloatSprintf__stdZformatfloat_127)(NimStringV2* result__PEHM0fGXU1QVA9b2Xxirrcw, NF x__ugCbnjmaLPuaKR4rpo60JQ) {	
#line 99 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
tyArray__eVNFTutn6un5gcq48fQLdg buffer;	
#line 100 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
NI n;NIM_BOOL* nimErr_;	nimfr_("addFloatSprintf", "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");{nimErr_ = nimErrorFlag();
#line 100 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	nimln_(100, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 100 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	n = writeFloatToBufferSprintf__stdZformatfloat_80(buffer, x__ugCbnjmaLPuaKR4rpo60JQ);	if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;
#line 101 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	nimln_(101, "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim");
#line 101 "/home/vincent/nim-1.9.3/nim/lib/std/formatfloat.nim"
	addCstringN__stdZformatfloat_5(result__PEHM0fGXU1QVA9b2Xxirrcw, ((NCSTRING) ((&buffer[(((NI)0))- 0]))), n);
	if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;	}BeforeRet_: ;
	popFrame();}
