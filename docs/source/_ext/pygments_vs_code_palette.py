from pygments.style import Style
from pygments.token import Token, Comment, Keyword, Name, String, Number, Operator, Punctuation, Generic, Literal

class VSCodeDarkPlusStyle(Style):
    """
    VS Code Dark+ palette for Pygments.
    """
    background_color = "#1e1e1e"
    default_style = ""

    styles = {
        # Root token: sets color in `.highlight { background: ...; color: ... }`
        # Without this, bare text (bash commands, unlabelled blocks) inherits the
        # light-mode dark-gray from github-light, making it invisible on #1e1e1e.
        Token:             "#b59b7d",

        # Literal scalars (e.g. yaml plain values): overrides light-mode #953800 rust-red
        Literal:           "#d4d4d4",

        # Base
        Name:              "#9cdcfe",
        Punctuation:       "#d4d4d4",
        Operator:          "#d4d4d4",

        # Comments
        Comment:           "italic #6a9955",

        # Keywords
        Keyword:          "#569cd6",
        Keyword.Reserved: "#c586c0",

        # Types / classes (Name.Class == .nc)
        Name.Class:        "#4ec9b0",
        Name.Namespace:    "#4ec9b0",
        Name.Exception:    "#4ec9b0",

        # Functions (Name.Function == .nf)
        Name.Function:     "#dcdcaa",

        # Variables / attributes
        Name.Variable:     "#9cdcfe",
        Name.Attribute:    "#9cdcfe",

        # Strings
        String:            "#ce9178",

        # Numbers
        Number:            "#b5cea8",

        # Preprocessor / directives
        Comment.Preproc:   "#c586c0",
        Comment.PreprocFile:"#c586c0",

        # Generic (optional)
        Generic.Heading:   "bold #d4d4d4",
        Generic.Subheading:"bold #d4d4d4",

        # Make `true/false/nullptr` look like keywords
        Name.Builtin: "#569cd6",        # `true`, `false`, `nullptr` in many lexers
        Name.Builtin.Pseudo: "#569cd6",
    }