def fact(N,depth=0):
    if N == 1:
        return(1)
    res=N*fact(N-1,depth+1)
    return(res)
print(fact(int(input())))


if len(A)<len(B):
    for g in range (len(B)+1):
        if g in A and g in B:
            res *= g
            g -= 1
        elif g not in A and g in B:
            res *= g
            g -= 1
        elif g in A and g not in B:
            res *= g
            g -= 1

if len(A)<len(B):

    elif N[0] != N[-1]:
    for i in range(max(N)):
        if min(N) * i - max(N) == res:
            x = i
            y = -1
        elif max(N) - min(N) * i == res:
            y = 1
            x = -i