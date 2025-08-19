from connect4.connect_state import ConnectState
from itertools import count
import heapq

class Path:
    def __init__(self, n, parent=None, g=0, a=None):
        # Inicializar variables
        self.head = n
        self.parent = parent
        self.a = a
        self.g = g
    
    def decode(self):
        """

        Returns:
            list: List of actions along the path from root to head
        """
        actions = []
        p = self
        while p is not None:
            if p.a is not None:
                actions.append(p.a)
            p = p.parent
        actions.reverse()

        return actions # Returns list of actions along the path from root to head

class UniformCostSearch:
    
    def __init__(self, n0, succ, star, cost):
        # Inicializar variables
        self._n0 = n0
        self._succ = succ
        self._star = star
        self._cost = cost

        # Estructuras UCS
        self._closed = set()                    
        self._open_heap = []                    
        self._open_by_head = {}                 
        self._counter = count()                 

        # Mejor solución
        self._best_goal_on_open = None
        self._solved_path = None

        # Inicializar OPEN
        root = Path(n0, parent=None, g=0, a=None)
        self._open_by_head[n0] = root
        heapq.heappush(self._open_heap, (0, next(self._counter), root))

    @property
    def open(self):
        """

        Returns:
            collection: The collection of paths on OPEN
        """
        return list(self._open_by_head.values()) # Returns the collection of paths on OPEN
    
    @property
    def closed(self):
        """

        Returns:
            collection: The collection of nodes on CLOSED
        """
        return set(self._closed) # Returns the collection of nodes on CLOSED

    @property
    def is_active(self):
        """

        Tells whether the algorithm is still active, which is the case unless an optimal solution has been found or OPEN is empty

        Returns:
            bool: True if the algorithm can still be stepped
        """
        no_solution_yet = self._solved_path is None
        open_not_empty = len(self._open_by_head) > 0 or len(self._open_heap) > 0 # OPEN tiene más elementos, true

        return no_solution_yet and open_not_empty # Returns true if the algorithm can still be stepped
    
    @property
    def best_known_solution(self):
        """

        Returns:
            list: list of actions associated with the best known solution
        """
        if self._solved_path is not None:
            return self._solved_path.decode()
        if self._best_goal_on_open is not None:
            return self._best_goal_on_open.decode()
        
        return None # Returns list of actions associated with the best known solution
    
    @property
    def optimality_proven(self):
        """

        Returns:
            bool: True iff the best known solution has been proven to be optimal (because it comes from an expanded path to a goal node)
        """
        return self._solved_path is not None # Returns true iff the best known solution has been proven to be optimal
    
    def step(self):
        """
        Executes a single step of the main loop of Uniform Cost Search

        Returns:
            Path: the expanded path
        """
        if not self.is_active:
            return None # Si ya se encontró solución óptima o OPEN está vacío, no hay más pasos

        p = self._pop_min_open() # Camino con menor costo acumulado
        if p is None:
            return None # OPEN está vacío, no se puede expandir

        if self._star(p.head): # Si el head es goal, se termina y se guarda la solución
            self._solved_path = p
            return p

        self._closed.add(p.head) # Nodo a CLOSED
        self._open_by_head.pop(p.head, None)

        # Expandir sucesores del nodo actual
        for (a, nprime) in self._succ(p.head):
            if nprime in self._closed:
                continue
            new_g = p.g + self._cost(p.head, nprime)

            existing = self._open_by_head.get(nprime)
            if existing is None:
                child = Path(nprime, parent=p, g=new_g, a=a)
                self._open_by_head[nprime] = child
                heapq.heappush(self._open_heap, (child.g, next(self._counter), child))
                if self._star(nprime):
                    if (self._best_goal_on_open is None) or (child.g < self._best_goal_on_open.g):
                        self._best_goal_on_open = child
            else: # El sucesor ya estaba en OPEN, se aplica "parent discarding"
                if new_g < existing.g:
                    existing.parent = p
                    existing.g = new_g
                    existing.a = a
                    heapq.heappush(self._open_heap, (existing.g, next(self._counter), existing))
                    if self._star(nprime):
                        if (self._best_goal_on_open is None) or (existing.g < self._best_goal_on_open.g):
                            self._best_goal_on_open = existing

        return p # Returns the expanded path
        
def get_ucs_arguments_for_tour_problem(locations, costs, n0, goal):
    """

    Args:
        locations (list): List of strings with names of locations
        costs (dict): depth-2 dictionary where costs[a][b] is the cost to travel from location a to b
        n0 (str): starting location
        goal (str): goal location to reach
    
    Returns:
        dict: Dictionary with keys "n0", "succ", "star", "cost"
    """
    def succ(n):
        succesors = []
        for m, c in costs.get(n, {}).items():
            succesors.append((m, m))

        return succesors

    def star(n):
        # goal location to reach
        return n == goal

    def cost(n, nprime):
        # depth-2 dictionary where costs[a][b] is the cost to travel from location a to b
        return costs[n][nprime]

    return { # Returns dictionary with keys "n0", "succ", "star", "cost"
        "n0": n0,
        "succ": succ,
        "star": star,
        "cost": cost,
    }

def get_ucs_arguments_for_deterministic_connect4_player(opponent):
    """
    Args:
        opponent (func): Function that reflects how the opponent plays in any situation

    Returns:
        dict: Dictionary with keys "n0", "succ", "star", "cost"
    """
    s0 = ConnectState() # Estado inicial: El oponente rojo (= -1) juega primero
    if not s0.is_final():
        try:
            opp_col = opponent(s0)
            if s0.is_applicable(opp_col):
                s0 = s0.transition(opp_col)
        except Exception:
            pass

    # Sucesores
    def succ(state: ConnectState):
        if state.is_final():
            return []

        succesors = []
        for a in state.get_free_cols():  # columnas validas
            after_us = state.transition(a)

            # ¿Se gana inmediatamente con la jugada?
            if after_us.is_final() and after_us.get_winner() == 1:
                succesors.append((a, after_us))
                continue

            # Si no, juega el oponente
            s = after_us
            if not s.is_final():
                try:
                    col = opponent(s)
                    if s.is_applicable(col):
                        s = s.transition(col)
                except Exception:
                    pass

            succesors.append((a, s))

        return succesors

    # Objetivo: Amarillo (= 1) gana
    def star(state: ConnectState):
        return state.is_final() and state.get_winner() == 1

    # Costo uniforme por cada jugada
    def cost(s, s2):
        return 1

    return { # Returns dictionary with keys "n0", "succ", "star", "cost"
        "n0": s0,
        "succ": succ,
        "star": star,
        "cost": cost,
    }