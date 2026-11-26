/** @file    tkernel.h  (TEST STUB)
 *  @brief   Minimal stand-in for the micro T-Kernel API that the firmware
 *           files under test use. The behaviour is in stub_kernel.c.
 *
 *  Only the calls the car code makes are provided. They follow the
 *  T-Kernel rules that matter to the tests: event flag patterns, and
 *  message buffers that store each message as a 4-byte length header plus
 *  the message rounded up to 4 bytes.
 */
#ifndef STUB_TKERNEL_H
#define STUB_TKERNEL_H

#include <stddef.h>
#include <stdint.h>

typedef int          ID;
typedef int          INT;
typedef unsigned int UINT;
typedef unsigned char UB;
typedef int          ER;
typedef int          PRI;
typedef int          TMO;
typedef unsigned int RELTIM;
typedef size_t       SZ;
typedef void       (*FP)(void);
typedef void *       VP;

#define E_OK         (0)
#define E_PAR        (-17)
#define E_ID         (-18)
#define E_TMOUT      (-50)

#define TA_TFIFO     (0x00U)
#define TA_WMUL      (0x08U)
#define TA_INHERIT   (0x02U)
#define TA_HLNG      (0x01U)
#define TA_RNG3      (0x0300U)

#define TWF_ORW      (0x01U)
#define TWF_CLR      (0x10U)
#define TWF_BITCLR   (0x20U)

#define TMO_POL      (0)
#define TMO_FEVR     (-1)

typedef struct { void * exinf; UINT flgatr; UINT iflgptn; UB dsname[8]; } T_CFLG;
typedef struct { void * exinf; ID wtsk; UINT flgptn; } T_RFLG;
typedef struct { void * exinf; UINT mbfatr; SZ bufsz; INT maxmsz; UB dsname[8]; void * bufptr; } T_CMBF;
typedef struct { void * exinf; UINT mtxatr; PRI ceilpri; UB dsname[8]; } T_CMTX;
typedef struct { void * exinf; UINT tskatr; FP task; PRI itskpri; SZ stksz; } T_CTSK;
typedef struct { void * exinf; UINT intatr; FP inthdr; } T_DINT;

ID tk_cre_flg (T_CFLG const * p_cflg);
ER tk_set_flg (ID flgid, UINT setptn);
ER tk_clr_flg (ID flgid, UINT clrptn);
ER tk_wai_flg (ID flgid, UINT waiptn, UINT wfmode, UINT * p_flgptn, TMO tmout);
ER tk_ref_flg (ID flgid, T_RFLG * p_rflg);

ID tk_cre_mbf (T_CMBF const * p_cmbf);
ER tk_snd_mbf (ID mbfid, void const * p_msg, INT msgsz, TMO tmout);
INT tk_rcv_mbf (ID mbfid, void * p_msg, TMO tmout);

ID tk_cre_mtx (T_CMTX const * p_cmtx);
ER tk_loc_mtx (ID mtxid, TMO tmout);
ER tk_unl_mtx (ID mtxid);

ER tk_dly_tsk (RELTIM dlytim);
ER tk_slp_tsk (TMO tmout);

#endif /* STUB_TKERNEL_H */
