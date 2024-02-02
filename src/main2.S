# Program used to test mul
#

            .text

start:      li    t1, 1
            mv    a0, t1
            mul   a0, a0, t1
            mul   a0, a0, t1
            beqz  zero,  start
