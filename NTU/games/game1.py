# Fox, chiken, farmer, grain

a = set({2,4})
b = set({3,2})

options = [None,'g','c','f']

# Initial set = [f,c,fr,g]

state = [set({'f','c','g'}),set({}),0]
states = []


state = {'g':0,'c':0,'f':0,'fr':0}

def act(sta,track):

    che = True
    for s in states:
        # print(st[0],s[0],st[1],s[1],st[2],s[2])
        if sta['g'] == s['g'] and sta['c'] == s['c'] and sta['f'] == s['f'] and sta['fr']==s['fr']:
            che = False


    if che:
        print(sta,track)

        states.append({'g':sta['g'],'c':sta['c'],'f':sta['f'],'fr':sta['fr']})

        # Leave boat
        for i in ['g','c','f']:
            for j in options:

                state_copy = {'g': sta['g'], 'c': sta['c'], 'f': sta['f'], 'fr': sta['fr']}
                if j!=None:
                    state_copy[j] = 2
                if state_copy[i] == 2:
                    state_copy[i] = state_copy['fr']



                if state_copy['g'] != state_copy['c'] and state_copy['f'] != state_copy['c']:
                    state_copy['fr'] = 1-state_copy['fr']
                    track_copy = list(track)
                    track_copy.append(state_copy)
                    act(state_copy,track_copy)


def check(state):
    check = True
    if 'g' in state[0] and 'c' in state[0]:
        check = False
    if 'f' in state[0] and 'c' in state[0]:
        check = False
    if 'g' in state[1] and 'c' in state[1]:
        check = False
    if 'f' in state[1] and 'c' in state[1]:
        check = False
    return  check
act(state,[])



