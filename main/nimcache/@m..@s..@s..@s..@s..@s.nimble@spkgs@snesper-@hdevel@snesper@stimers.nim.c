/* Generated by Nim Compiler v1.6.12 */
#define NIM_INTBITS 32

#include "nimbase.h"
#include "esp_timer.h"
#include <freertos/FreeRTOS.h>
                   #include "freertos/task.h" 
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
  #  define nimfr_(proc, file) \
      TFrame FR_; \
      FR_.procname = proc; FR_.filename = file; FR_.line = 0; FR_.len = 0; nimFrame(&FR_);

  #  define nimfrs_(proc, file, slots, length) \
      struct {TFrame* prev;NCSTRING procname;NI line;NCSTRING filename; NI len; VarSlot s[slots];} FR_; \
      FR_.procname = proc; FR_.filename = file; FR_.line = 0; FR_.len = length; nimFrame((TFrame*)&FR_);

  #  define nimln_(n, file) \
      FR_.line = n; FR_.filename = file;
  N_LIB_PRIVATE N_NIMCALL(NU64, delayMillis__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_163)(NU64 ms);
N_LIB_PRIVATE N_NIMCALL(NU64, millis__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_150)(void);
N_LIB_PRIVATE N_NIMCALL(NU64, micros__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_136)(void);
static N_INLINE(NU64, microsRaw__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_134)(void);
static N_INLINE(void, nimFrame)(TFrame* s);
N_LIB_PRIVATE N_NOINLINE(void, callDepthLimitReached__system_5032)(void);
static N_INLINE(void, popFrame)(void);
static N_INLINE(NIM_BOOL*, nimErrorFlag)(void);
N_LIB_PRIVATE N_NIMCALL(NU32, toTicks__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZgeneral_37)(NU64 ts);
N_LIB_PRIVATE N_NIMCALL(NU64, delayMicros__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_158)(NU64 us);
extern NIM_THREADVAR TFrame* framePtr__system_4513;
extern NIM_THREADVAR TFrame* framePtr__system_4513;
extern NIM_THREADVAR TFrame* framePtr__system_4513;
extern NIM_THREADVAR TFrame* framePtr__system_4513;
extern NIM_THREADVAR TFrame* framePtr__system_4513;
extern NIM_THREADVAR TFrame* framePtr__system_4513;
extern NIM_THREADVAR NIM_BOOL nimInErrorMode__system_4861;

#line 549 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
static N_INLINE(void, nimFrame)(TFrame* s) {
#line 550 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
	{
#line 550 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
		if (!(framePtr__system_4513 == ((TFrame*) NIM_NIL))) goto LA3_;

#line 551 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
		(*s).calldepth = ((NI16) 0);	}
	goto LA1_;
	LA3_: ;
	{
#line 554 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"

#line 554 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
		(*s).calldepth = (NI16)((*framePtr__system_4513).calldepth + ((NI16) 1));	}
	LA1_: ;

#line 556 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
	(*s).prev = framePtr__system_4513;
#line 557 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
	framePtr__system_4513 = s;
#line 558 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
	{
#line 558 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
		if (!((*s).calldepth == ((NI16) 2000))) goto LA8_;

#line 558 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"

#line 558 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
		callDepthLimitReached__system_5032();
	}
	LA8_: ;
}

#line 99 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
static N_INLINE(void, popFrame)(void) {
#line 100 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
	framePtr__system_4513 = (*framePtr__system_4513).prev;}

#line 41 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
static N_INLINE(NU64, microsRaw__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_134)(void) {	NU64 result;	NI64 T1_;	nimfr_("microsRaw", "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");{	result = (NU64)0;
#line 42 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(42, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 42 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"

#line 42 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"

#line 42 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	T1_ = (NI64)0;	T1_ = esp_timer_get_time();	result = ((NU64) (T1_));	goto BeforeRet_;
	}BeforeRet_: ;
	popFrame();	return result;}

#line 423 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
static N_INLINE(NIM_BOOL*, nimErrorFlag)(void) {	NIM_BOOL* result;	result = (NIM_BOOL*)0;
#line 424 "/home/vincent/.choosenim/toolchains/nim-1.6.12/lib/system/excpt.nim"
	result = (&nimInErrorMode__system_4861);	return result;}

#line 44 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
N_LIB_PRIVATE N_NIMCALL(NU64, micros__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_136)(void) {	NU64 result;NIM_BOOL* nimErr_;	nimfr_("micros", "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");{nimErr_ = nimErrorFlag();	result = (NU64)0;
#line 45 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(45, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 45 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"

#line 45 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"

#line 45 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	result = microsRaw__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_134();	if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;	goto BeforeRet_;
	}BeforeRet_: ;
	popFrame();	return result;}

#line 47 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
N_LIB_PRIVATE N_NIMCALL(NU64, millis__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_150)(void) {	NU64 result;	NU64 T1_;NIM_BOOL* nimErr_;	nimfr_("millis", "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");{nimErr_ = nimErrorFlag();	result = (NU64)0;
#line 48 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(48, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 48 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"

#line 48 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"

#line 48 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"

#line 48 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	T1_ = (NU64)0;	T1_ = micros__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_136();	if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;	result = (NU64)((NU64)(T1_) / (NU64)(1000ULL));	goto BeforeRet_;
	}BeforeRet_: ;
	popFrame();	return result;}

#line 60 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
N_LIB_PRIVATE N_NIMCALL(NU64, delayMicros__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_158)(NU64 us) {	NU64 result;	NU64 curr;	NU64 target;NIM_BOOL* nimErr_;	nimfr_("delayMicros", "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");{nimErr_ = nimErrorFlag();	result = (NU64)0;
#line 61 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(61, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");	{
#line 61 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
		if (!(us == 0ULL)) goto LA3_;

#line 62 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
		nimln_(62, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 62 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
		result = 0ULL;		goto BeforeRet_;
	}
	LA3_: ;

#line 64 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(64, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 64 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	curr = microsRaw__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_134();	if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;
#line 65 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(65, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 65 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	target = (NU64)((NU64)(curr) + (NU64)(us));
#line 66 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(66, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");	{
#line 66 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
		if (!((NU64)(target) < (NU64)(curr))) goto LA7_;
		{
#line 67 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
			nimln_(67, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");			while (1) {
#line 67 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
				if (!((NU64)(target) < (NU64)(curr))) goto LA10;

#line 68 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
				nimln_(68, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 68 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
				curr = microsRaw__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_134();				if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;			} LA10: ;
		}
	}
	LA7_: ;
	{
#line 70 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
		nimln_(70, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");		while (1) {
#line 70 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
			if (!((NU64)(curr) < (NU64)(target))) goto LA12;

#line 71 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
			nimln_(71, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 71 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
			curr = microsRaw__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_134();			if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;		} LA12: ;
	}

#line 73 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(73, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 73 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"

#line 73 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	result = (NU64)((NU64)(target) - (NU64)(curr));	goto BeforeRet_;
	}BeforeRet_: ;
	popFrame();	return result;}

#line 75 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
N_LIB_PRIVATE N_NIMCALL(NU64, delayMillis__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_163)(NU64 ms) {	NU64 result;	NU64 colontmpD_;	NU64 start;	NU32 ticks;	NU64 stop;NIM_BOOL* nimErr_;	nimfr_("delayMillis", "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");{nimErr_ = nimErrorFlag();	result = (NU64)0;	colontmpD_ = (NU64)0;
#line 76 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(76, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 76 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	start = millis__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_150();	if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;
#line 77 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(77, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 77 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	ticks = toTicks__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZgeneral_37(ms);	if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;
#line 78 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(78, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");	{
#line 78 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
		if (!((NU32)(((NU32) 0)) < (NU32)(ticks))) goto LA3_;

#line 79 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
		nimln_(79, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 79 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
		vTaskDelay(ticks);
	}
	goto LA1_;
	LA3_: ;
	{
#line 81 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
		nimln_(81, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 81 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"

#line 81 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
		colontmpD_ = delayMicros__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_158((NU64)((NU64)(1000ULL) * (NU64)(ms)));		if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;	}
	LA1_: ;

#line 83 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(83, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 83 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	stop = millis__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_150();	if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;
#line 84 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(84, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 84 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"

#line 84 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	result = (NU64)((NU64)(stop) - (NU64)(start));	goto BeforeRet_;
	}BeforeRet_: ;
	popFrame();	return result;}

#line 86 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
N_LIB_PRIVATE N_NIMCALL(void, delay__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_169)(NU64 ts) {	NU64 T1_;NIM_BOOL* nimErr_;	nimfr_("delay", "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");{nimErr_ = nimErrorFlag();
#line 86 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	nimln_(86, "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim");
#line 86 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"

#line 86 "/home/vincent/.nimble/pkgs/nesper-#devel/nesper/timers.nim"
	T1_ = (NU64)0;	T1_ = delayMillis__OOZOOZOOZOOZOOZOnimbleZpkgsZnesper4535develZnesperZtimers_163(ts);	if (NIM_UNLIKELY(*nimErr_)) goto BeforeRet_;	(void)(T1_);
	}BeforeRet_: ;
	popFrame();}
