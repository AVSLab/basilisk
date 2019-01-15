\NeedsTeXFormat{LaTeX2e}[1994/06/01]
\ProvidesClass{IAC}[2015/09/09 v1.0 AVS Laboratory document class]
\typeout{TYPESETTING in AVS Lab Technical Report...}

\LoadClass[11pt]{article}

%
%	Load in required packages
%
\RequirePackage{graphicx}
\usepackage{natbib}
\RequirePackage{bm}
\RequirePackage{overcite}
\RequirePackage{amssymb}
\RequirePackage{amsmath}
\RequirePackage{subfigure}
\RequirePackage{longtable}
\RequirePackage{mathabx}
\RequirePackage{mathtools}
\RequirePackage{color}
\RequirePackage[normalem]{ulem}
\RequirePackage{lastpage}
\usepackage{listings}
\usepackage{xcolor}
%\RequirePackage[original,runin]{abstract}
\RequirePackage[total={6.5in,9in},top=1in, left=1in]{geometry}
\RequirePackage[colorlinks=false,hyperfootnotes=false]{hyperref}
\RequirePackage{url}
\RequirePackage{titling}
\RequirePackage[compact]{titlesec}
\RequirePackage[margin=0pt,font=small,labelfont=bf,labelsep=colon,indention=1.5em]{caption}
%\RequirePackage{ragged2e}
\RequirePackage[perpage,ragged,hang]{footmisc}
\RequirePackage[absolute]{textpos}

\renewcommand*{\familydefault}{\sfdefault}


%
%	setup header and footers
%
\RequirePackage{fancyhdr}
\fancyhead{}
\fancyfoot{}
\fancyhead[L]{\footnotesize AVS-\ReportYear-\ReportNumber\ (Version \Version) }
\fancyhead[R]{\footnotesize  Page \thepage \ of \pageref{LastPage}}
\renewcommand{\headrulewidth}{0pt}
\renewcommand{\footrulewidth}{0pt}



%
%	setup makeCover command
%
\newcommand{\makeCover}{

\begin{textblock*}{2.5in}(2.725in,1in)
	\includegraphics[width=2.5in]{Figures/AVSlogo5.pdf}
\end{textblock*}

\begin{textblock*}{6.in}(1.25in,3in)
{
\centering

\large\bfseries  AVS Laboratory Technical Report\\
\normalsize\normalfont AVS-\ReportYear-\ReportNumber\ (Version \Version)\\
 \Release \\
\vskip 8em
{\LARGE\bfseries\MakeUppercase{\ReportTitle}}
\vskip 3em 
{\normalsize\normalfont
\ReportAuthor \\
{\itshape \AuthorTitle} \\
\href{mailto:\AuthorEmail}{\color{black}\AuthorEmail}
\vskip 4em
\today \\
}
\vskip 4em
{\bfseries Abstract:\quad}\summary
}

\end{textblock*}
}

%
%	setup float environment spacing
%
\setlength\textfloatsep{5pt}
\setlength{\abovecaptionskip}{3pt}
\renewcommand{\figurename}{Fig.}


%
%	Rename the Bibliography section label
%
\renewcommand{\refname}{REFERENCES}

%
%	Miscellaneous definitions
%

