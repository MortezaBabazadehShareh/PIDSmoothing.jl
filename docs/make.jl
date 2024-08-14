using Documenter
using PIDSmoothing
#=
makedocs(
    #sitename = "PIDSmoothing.jl Documentation",
    #modules = [PIDSmoothing],
    pages = [
        #"Home" => "index.md",
        #"Functions" => "functions.md",
        modules = [PIDSmoothing],
        sitename = "PIDSmoothing.jl Documentation",
        pages = Any[
            "PIDSmoothing" => "index.md"
        ]
    ],
    format = Documenter.HTML()
)=#

makedocs(
    modules = [PIDSmoothing],
    sitename = "PIDSmoothing",
    pages = Any[
        "PIDSmoothing" => "index.md"
        ],
       
    )
format = Documenter.HTML()

