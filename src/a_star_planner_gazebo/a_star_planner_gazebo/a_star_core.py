import heapq, numpy as np

def inflate_obstacles(occ, inflate):
    if inflate <= 0: return occ
    from scipy.ndimage import grey_dilation
    return grey_dilation(occ, size=(inflate*2+1, inflate*2+1))

def a_star_costmap(cost, start, goal, occ_block=50):
    H, W = cost.shape
    sx, sy = start; gx, gy = goal
    def inb(x,y): return 0<=x<W and 0<=y<H
    def h(x,y): return abs(x-gx) + abs(y-gy)
    open_=[]; heapq.heappush(open_, (0,(sx,sy)))
    came={}; g={(sx,sy):0.0}
    nbrs=[(1,0),(-1,0),(0,1),(0,-1)]
    while open_:
        _,(x,y)=heapq.heappop(open_)
        if (x,y)==(gx,gy): break
        for dx,dy in nbrs:
            nx,ny=x+dx,y+dy
            if not inb(nx,ny): continue
            c = cost[ny,nx]
            if c>=occ_block:  # considered blocked
                continue
            ng = g[(x,y)] + (1.0 + c/100.0)  # base cost + learned/occ penalty
            if (nx,ny) not in g or ng < g[(nx,ny)]:
                g[(nx,ny)] = ng
                came[(nx,ny)] = (x,y)
                heapq.heappush(open_, (ng+h(nx,ny), (nx,ny)))
    if (gx,gy) not in came: return []
    path=[(gx,gy)]
    while path[-1]!=(sx,sy):
        path.append(came[path[-1]])
    path.reverse()
    return path
