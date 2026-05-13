from pygments.lexers import CppLexer
from pygments.filter import Filter
from pygments.token import Name, Punctuation, Keyword

class SymXSemanticHintFilter(Filter):
    def __init__(self, **options):
        super().__init__(**options)
        self.types = set(options.get("types", []))
        self.funcs = set(options.get("funcs", []))
        self.heuristic_calls = bool(options.get("heuristic_calls", True))
        self.pink_keywords = {"for", "if", "while", "switch", "return"}

    def filter(self, lexer, stream):
        prev = None
        for ttype, value in stream:
            if prev is None:
                prev = (ttype, value)
                continue

            ptype, pval = prev

            if ptype in Keyword and pval in self.pink_keywords:
                ptype = Keyword.Reserved

            if ptype is Name:
                if pval in self.types:
                    ptype = Name.Class
                elif pval in self.funcs:
                    ptype = Name.Function
                elif self.heuristic_calls and ttype is Punctuation and value == "(":
                    ptype = Name.Function

            yield (ptype, pval)
            prev = (ttype, value)

        if prev is not None:
            ptype, pval = prev
            if ptype in Keyword and pval in self.pink_keywords:
                ptype = Keyword.Reserved
            if ptype is Name:
                if pval in self.types:
                    ptype = Name.Class
                elif pval in self.funcs:
                    ptype = Name.Function
            yield (ptype, pval)


class SymXCppLexer(CppLexer):
    name = "C++ (SymX)"
    aliases = ["cpp", "c++"]

    def __init__(self, **options):
        super().__init__(**options)
        self.add_filter(
            SymXSemanticHintFilter(
                types=[
                    "Scalar", "Vector", "Matrix", "Index",
                    "MappedWorkspace", "CompiledInLoop",
                    "std", "vector", "Eigen", "Vector3d", 
                    "array", "int32_t", "View", "string",
                    "INPUT_FLOAT", "COMPILED_FLOAT", "OUTPUT_FLOAT",
                    "__m256d", "Info", "CALLBACK_T", "function", "Triplet",
                    "T", "FEM_Element", "Context", "spContext", "spGlobalPotential",
                    "GlobalPotential", "Verbosity", "NewtonsMethod", "ProjectionToPD",
                    "SolverReturn", "LabelledConnectivity", "numeric_limits", "LinearSolver",
                    "Workspace", "Compiled", "DiffCache", "Compilation", "BlockedSparseMatrix", "bsm",
                    "Ordering", "RowMajor", "Tet4", "Hex8", "Tet10", "Hex27", "Element", "VectorXd", "symx"
                ],
                funcs=[
                    "PLACEHOLDER",
                ],
                heuristic_calls=True,
            )
        )