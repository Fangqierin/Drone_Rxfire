from enum import Enum
from textwrap import wrap, dedent
from itertools import chain

from pyknow import *

class Nationality(Enum):
    englishman = 0
    spaniard = 1
    ukrainian = 2
    norwegian = 3
    japanese = 4
class Color(Enum):
    red = 0
    green = 1
    ivory = 2
    yellow = 3
    blue = 4

class Pet(Enum):
    dog = 0
    snails = 1
    fox = 2
    horse = 3
    zebra = 4


class Drink(Enum):
    water = 0
    coffee = 1
    milk = 2
    orange_juice = 3
    tea = 4
class Smokes(Enum):
    old_golds = 0
    kools = 1
    chesterfields = 2
    lucky_strikes = 3
    parliaments = 4


class Value(Fact):
    pass


class Solution(Fact):
    pass



class Zebra(KnowledgeEngine):
    @DefFacts()
    def startup(self):
        print(dedent("""
            There are five houses, each of a different color, inhabited by men of
            different nationalities, with different pets, drinks, and cigarettes.
            
            The Englishman lives in the red house.  The Spaniard owns the dog.
            The ivory house is immediately to the left of the green house, where
            the coffee drinker lives.  The milk drinker lives in the middle house.
            The man who smokes Old Golds also keeps snails.  The Ukrainian drinks
            tea.  The Norwegian resides in the first house on the left.  The
            Chesterfields smoker lives next door to the fox owner.  The Lucky
            Strike smoker drinks orange juice.  The Japanese smokes Parliaments.
            The horse owner lives next to the Kools smoker, whose house is yellow.
            The Norwegian lives next to the blue house.
            
            Now, who drinks water?  And who owns the zebra?
            """))

        for x in chain(Nationality, Color, Pet, Drink, Smokes):
            #print(f"FQ {x}")
            yield Value(x)

    @Rule(AS.f << Value(MATCH.v))
    def generate_combinations(self, f, v): 
        print(f"FQ See value {f},{v}")
        self.retract(f)
        self.declare(*[Fact(v, x) for x in range(1, 6)])

    @Rule(
        # The Englishman lives in the red house.
        Fact(Nationality.englishman, MATCH.n1),
        Fact(Color.red, MATCH.c1 & MATCH.n1),
        
        # The Spaniard owns the dog.
        Fact(Nationality.spaniard, MATCH.n2 & ~MATCH.n1),
        Fact(Pet.dog, MATCH.p1 & MATCH.n2),
        
        # The ivory house is immediately to the left of the green house,
        # where the coffee drinker lives.
        Fact(Color.ivory, MATCH.c2 & ~MATCH.c1),
        Fact(Color.green, MATCH.c3 & ~MATCH.c2 & ~MATCH.c1),
        TEST(lambda c3, c2: c3 == (c2 + 1)),
        Fact(Drink.coffee, MATCH.d1 & MATCH.c3),

        # The milk drinker lives in the middle house.
        Fact(Drink.milk, MATCH.d2 & ~MATCH.d1 & L(3)),
        
        # The man who smokes Old Golds also keeps snails.
        Fact(Smokes.old_golds, MATCH.s1),
        Fact(Pet.snails, MATCH.p2 & ~MATCH.p1 & MATCH.s1),
        
        # The Ukrainian drinks tea.
        Fact(Nationality.ukrainian, MATCH.n3 & ~MATCH.n2 & ~MATCH.n1),
        Fact(Drink.tea, MATCH.d3 & ~MATCH.d2 & ~MATCH.d1 & MATCH.n3),
        
        # The Norwegian resides in the first house on the left.
        Fact(Nationality.norwegian, MATCH.n4 & ~MATCH.n3 & ~MATCH.n2 & ~MATCH.n1 & L(1)),
        
        # Chesterfields smoker lives next door to the fox owner.
        Fact(Smokes.chesterfields, MATCH.s2 & ~MATCH.s1),
        Fact(Pet.fox, MATCH.p3 & ~MATCH.p2 & ~MATCH.p1),
        TEST(lambda s2, p3: (s2==p3-1) or (s2==p3+1)),
        
        # The Lucky Strike smoker drinks orange juice.
        Fact(Smokes.lucky_strikes, MATCH.s3 & ~MATCH.s2 & ~MATCH.s1),
        Fact(Drink.orange_juice, MATCH.d4 & ~MATCH.d3 & ~MATCH.d2 & ~MATCH.d1 & MATCH.s3),
        
        # The Japanese smokes Parliaments
        Fact(Nationality.japanese, MATCH.n5 & ~MATCH.n4 & ~MATCH.n3 & ~MATCH.n2 & ~MATCH.n1),
        Fact(Smokes.parliaments, MATCH.s4 & ~MATCH.s3 & ~MATCH.s2 & ~MATCH.s1 & MATCH.n5),
        
        # The horse owner lives next to the Kools smoker, 
        # whose house is yellow.
        Fact(Pet.horse, MATCH.p4 & ~MATCH.p3 & ~MATCH.p2 & ~MATCH.p1),
        Fact(Smokes.kools, MATCH.s5 & ~MATCH.s4 & ~MATCH.s3 & ~MATCH.s2 & ~MATCH.s1),
        TEST(lambda p4, s5: (p4==s5-1) or (p4==s5+1)),
        Fact(Color.yellow, MATCH.c4 & ~MATCH.c3 & ~MATCH.c2 & ~MATCH.c1 & MATCH.s5),
        
        # The Norwegian lives next to the blue house.
        Fact(Color.blue, MATCH.c5 & ~MATCH.c4 & ~MATCH.c3 & ~MATCH.c2 & ~MATCH.c1),
        TEST(lambda c5, n4: (c5==n4-1) or (c5==n4+1)),
        
        # Who drinks water?  And Who owns the zebra?
        Fact(Drink.water, MATCH.d5 & ~MATCH.d4 & ~MATCH.d3 & ~MATCH.d2 & ~MATCH.d1),
        Fact(Pet.zebra, MATCH.p5 & ~MATCH.p4 & ~MATCH.p3 & ~MATCH.p2 & ~MATCH.p1)
    )
    def find_solution(self, **match):
        print(f"FQ match {match}")
        self.declare(*[
            Solution(Nationality, 'englishman', match["n1"]),
            Solution(Color, 'red', match["c1"]),
            Solution(Nationality, 'spaniard', match["n2"]),
            Solution(Pet, 'dog', match["p1"]),
            Solution(Color, 'ivory', match["c2"]),
            Solution(Color, 'green', match["c3"]),
            Solution(Drink, 'coffee', match["d1"]),
            Solution(Drink, 'milk', match["d2"]) ,
            Solution(Smokes, 'old_golds', match["s1"]),
            Solution(Pet, 'snails', match["p2"]),
            Solution(Nationality, 'ukrainian', match["n3"]),
            Solution(Drink, 'tea', match["d3"]),
            Solution(Nationality, 'norwegian', match["n4"]),
            Solution(Smokes, 'chesterfields', match["s2"]),
            Solution(Pet, 'fox', match["p3"]),
            Solution(Smokes, 'lucky_strikes', match["s3"]),
            Solution(Drink, 'orange_juice', match["d4"]) ,
            Solution(Nationality, 'japanese', match["n5"]),
            Solution(Smokes, 'parliaments', match["s4"]),
            Solution(Pet, 'horse', match["p4"]) ,
            Solution(Smokes, 'kools', match["s5"]),
            Solution(Color, 'yellow', match["c4"]),
            Solution(Color, 'blue', match["c5"]),
            Solution(Drink, 'water', match["d5"]),
            Solution(Pet, 'zebra', match["p5"])])

    @Rule(
        AS.f1  << Solution(Nationality, MATCH.n1, 1),
        AS.f2  << Solution(Color, MATCH.c1, 1),
        AS.f3  << Solution(Pet, MATCH.p1, 1),
        AS.f4  << Solution(Drink, MATCH.d1, 1),
        AS.f5  << Solution(Smokes, MATCH.s1, 1),
        AS.f6  << Solution(Nationality, MATCH.n2, 2),
        AS.f7  << Solution(Color, MATCH.c2, 2),
        AS.f8  << Solution(Pet, MATCH.p2, 2),
        AS.f9  << Solution(Drink, MATCH.d2, 2),
        AS.f10 << Solution(Smokes, MATCH.s2, 2),
        AS.f11 << Solution(Nationality, MATCH.n3, 3),
        AS.f12 << Solution(Color, MATCH.c3, 3),
        AS.f13 << Solution(Pet, MATCH.p3, 3),
        AS.f14 << Solution(Drink, MATCH.d3, 3),
        AS.f15 << Solution(Smokes, MATCH.s3, 3),
        AS.f16 << Solution(Nationality, MATCH.n4, 4),
        AS.f17 << Solution(Color, MATCH.c4, 4),
        AS.f18 << Solution(Pet, MATCH.p4, 4),
        AS.f19 << Solution(Drink, MATCH.d4, 4),
        AS.f20 << Solution(Smokes, MATCH.s4, 4),
        AS.f21 << Solution(Nationality, MATCH.n5, 5),
        AS.f22 << Solution(Color, MATCH.c5, 5),
        AS.f23 << Solution(Pet, MATCH.p5, 5),
        AS.f24 << Solution(Drink, MATCH.d5, 5),
        AS.f25 << Solution(Smokes, MATCH.s5, 5)
    )
    def print_solution(self,
                       n1, n2, n3, n4, n5,
                       c1, c2, c3, c4, c5,
                       p1, p2, p3, p4, p5,
                       d1, d2, d3, d4, d5,
                       s1, s2, s3, s4, s5,
                       **fs):
        print(f"FQ see {self.facts}\n")
        for f in fs.values():
            self.retract(f)
        print(f"FQ after retract {self.facts}")
        print("HOUSE | %-11s | %-6s | %-6s | %-12s | %-13s" % ("Nationality", "Color", "Pet", "Drink", "Smokes"))
        print("--------------------------------------------------------------------")
        print("  1   | %-11s | %-6s | %-6s | %-12s | %-13s" % (n1, c1, p1, d1, s1))
        print("  2   | %-11s | %-6s | %-6s | %-12s | %-13s" % (n2, c2, p2, d2, s2))
        print("  3   | %-11s | %-6s | %-6s | %-12s | %-13s" % (n3, c3, p3, d3, s3))
        print("  4   | %-11s | %-6s | %-6s | %-12s | %-13s" % (n4, c4, p4, d4, s4))
        print("  5   | %-11s | %-6s | %-6s | %-12s | %-13s" % (n5, c5, p5, d5, s5))



z = Zebra()
z.reset()
z.run()












