/**
implement a path finding algorithm that can find a path from start to end in a 2D grid map, say 50x50 grid map that has some obstacles.
*/

// Micro:bit / MakeCode friendly A* implementation.
//
// Grid conventions:
// - grid[i][j] = 0 means free
// - grid[i][j] = 1 means obstacle
// - i is row index (y direction), j is column index (x direction)
//
// Output conventions:
// - Returned path is a list of cells [{i,j}, ...]
// - You can convert to metric waypoints ({x,y}) using `cellPathToWaypoints`.

interface Cell {
    i: number
    j: number
}

function inBounds(i: number, j: number, rows: number, cols: number): boolean {
    return i >= 0 && i < rows && j >= 0 && j < cols
}

function keyOf(i: number, j: number, cols: number): number {
    return i * cols + j
}

function cellFromKey(k: number, cols: number): Cell {
    const i = Math.idiv(k, cols)
    const j = k - i * cols
    return { i: i, j: j }
}

function manhattan(a: Cell, b: Cell): number {
    return Math.abs(a.i - b.i) + Math.abs(a.j - b.j)
}

function reconstructPath(cameFrom: number[], startKey: number, goalKey: number, cols: number): Cell[] {
    const out: Cell[] = []
    let cur = goalKey
    if (cur !== startKey && cameFrom[cur] === -1) {
        return []
    }
    while (cur !== -1) {
        out.push(cellFromKey(cur, cols))
        if (cur === startKey) break
        cur = cameFrom[cur]
    }
    out.reverse()
    return out
}

// A* search on a grid with 4-connected neighbors.
function astar(grid: number[][], start: Cell, goal: Cell): Cell[] {
    const rows = grid.length
    const cols = grid[0].length
    const n = rows * cols

    const startKey = keyOf(start.i, start.j, cols)
    const goalKey = keyOf(goal.i, goal.j, cols)

    const gScore: number[] = []
    const fScore: number[] = []
    const cameFrom: number[] = []
    const closed: boolean[] = []
    const inOpen: boolean[] = []
    const open: number[] = []

    for (let k = 0; k < n; k++) {
        gScore.push(1e9)
        fScore.push(1e9)
        cameFrom.push(-1)
        closed.push(false)
        inOpen.push(false)
    }

    gScore[startKey] = 0
    fScore[startKey] = manhattan(start, goal)
    open.push(startKey)
    inOpen[startKey] = true

    while (open.length > 0) {
        // Find the open node with the smallest fScore (linear scan).
        let bestIdx = 0
        let bestKey = open[0]
        let bestF = fScore[bestKey]
        for (let t = 1; t < open.length; t++) {
            const k = open[t]
            const f = fScore[k]
            if (f < bestF) {
                bestF = f
                bestKey = k
                bestIdx = t
            }
        }

        open.removeAt(bestIdx)
        inOpen[bestKey] = false

        if (bestKey === goalKey) {
            return reconstructPath(cameFrom, startKey, goalKey, cols)
        }

        closed[bestKey] = true

        const cur = cellFromKey(bestKey, cols)
        const ci = cur.i
        const cj = cur.j

        // 4 neighbors
        const neighI = [ci - 1, ci + 1, ci, ci]
        const neighJ = [cj, cj, cj - 1, cj + 1]

        for (let u = 0; u < 4; u++) {
            const ni = neighI[u]
            const nj = neighJ[u]
            if (!inBounds(ni, nj, rows, cols)) continue
            if (grid[ni][nj] !== 0) continue

            const nk = keyOf(ni, nj, cols)
            if (closed[nk]) continue

            const tentativeG = gScore[bestKey] + 1
            if (tentativeG < gScore[nk]) {
                cameFrom[nk] = bestKey
                gScore[nk] = tentativeG
                fScore[nk] = tentativeG + manhattan({ i: ni, j: nj }, goal)
                if (!inOpen[nk]) {
                    open.push(nk)
                    inOpen[nk] = true
                }
            }
        }
    }

    return []
}

// Simplify a cell path into turning points.
function simplifyPath(path: Cell[]): Cell[] {
    if (path.length <= 2) return path
    const out: Cell[] = []
    out.push(path[0])

    let prevDi = path[1].i - path[0].i
    let prevDj = path[1].j - path[0].j

    for (let k = 1; k < path.length - 1; k++) {
        const di = path[k + 1].i - path[k].i
        const dj = path[k + 1].j - path[k].j
        if (di !== prevDi || dj !== prevDj) {
            out.push(path[k])
            prevDi = di
            prevDj = dj
        }
    }

    out.push(path[path.length - 1])
    return out
}

// Convert cell centers to odometry-frame waypoints.
// If each grid cell is `cellSize` mm, then:
// - x corresponds to column j
// - y corresponds to row i
function cellPathToWaypoints(path: Cell[], cellSize: number): { x: number, y: number }[] {
    const wps: { x: number, y: number }[] = []
    for (let k = 0; k < path.length; k++) {
        const c = path[k]
        wps.push({
            x: (c.j + 0.5) * cellSize,
            y: (c.i + 0.5) * cellSize,
        })
    }
    return wps
}

// -------------------------
// Demo (10x10 grid example)
// -------------------------
// For a true 50x50 map, generate `grid` programmatically (recommended) rather than typing it.

const GRID: number[][] = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 0, 0, 1, 1, 0],
    [0, 0, 0, 1, 0, 1, 0, 0, 1, 0],
    [0, 1, 0, 1, 0, 1, 0, 0, 1, 0],
    [0, 1, 0, 0, 0, 1, 1, 0, 0, 0],
    [0, 1, 0, 1, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 1, 0, 1, 1, 0, 1, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 1, 1, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 1, 0],
]

const startCell: Cell = { i: 0, j: 0 }
const goalCell: Cell = { i: 9, j: 9 }
const CELL_SIZE = 100

const pathCells = astar(GRID, startCell, goalCell)
const pathSimple = simplifyPath(pathCells)
const waypoints = cellPathToWaypoints(pathSimple, CELL_SIZE)

// Print waypoints that you can copy into robotpu-pure-pursuit.ts
serial.writeLine("A* raw cells: " + pathCells.length)
serial.writeLine("A* simplified: " + pathSimple.length)
for (let k = 0; k < waypoints.length; k++) {
    serial.writeLine("wp " + k + ": x=" + waypoints[k].x + ", y=" + waypoints[k].y)
}