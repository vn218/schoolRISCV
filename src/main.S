# Program used to test mul
#

            .text

start:      mv    t0, zero
            li    t1, 1
            mv    a0, t1

fibonacci:  add   t0, t0, t1
            mv    a0, t0
            add   t1, t0, t1
            mv    a0, t1
            mul   a0, a0, t1
            mul   a0, a0, t1
            beqz  zero,  fibonacci
