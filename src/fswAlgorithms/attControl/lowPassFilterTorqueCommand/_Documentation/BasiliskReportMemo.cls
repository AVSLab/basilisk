\NeedsTeXFormat{LaTeX2e}[1994/06/01]
\ProvidesClass{BasiliskReportMemo}[2015/03/30 v1.0 AVS Lab Software document class]
\typeout{TYPESETTING in BSK Technical  Report...}

\LoadClass[titlepage,11pt]{article}

%
%	Load in required packages
%
\RequirePackage{graphicx}
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
%\RequirePackage[original,runin]{abstract}
\RequirePackage[total={6.5in,9in},top=1in, left=1in]{geometry}
\RequirePackage[colorlinks=true,linkcolor=MidnightBlue,
			urlcolor=MidnightBlue,
		        citecolor=MidnightBlue,hyperfootnotes=false]{hyperref}
\RequirePackage{url}
\RequirePackage{titling}
\RequirePackage[compact]{titlesec}
\RequirePackage[margin=0pt,font=small,labelfont=bf,labelsep=colon,indention=1.5em]{caption}
%\RequirePackage{ragged2e}
\RequirePackage[perpage,ragged,hang]{footmisc}
\RequirePackage[absolute]{textpos}

\renewcommand*{\familydefault}{\sfdefault}
\renewcommand*{\thefootnote}{\fnsymbol{footnote}}


%
%	setup header and footers
%
\RequirePackage{fancyhdr}
\fancyhead{}
\fancyfoot{}
\fancyhead[L]{\footnotesize Doc. ID: Basilisk-\ModuleName}
\fancyhead[R]{\footnotesize Page \thepage \ of \pageref{LastPage}}
\renewcommand{\headrulewidth}{0pt}
\renewcommand{\footrulewidth}{0pt}

\setcounter{secnumdepth}{5}
\setcounter{tocdepth}{5}


%
%	setup makeCover command
%
\newcommand{\makeCover}{

\begin{textblock*}{6.625in}(2.25in,0.975in)
	\includegraphics[width=3.5in]{Figures/Basilisk-Logo.pdf}
\end{textblock*}



\begin{textblock*}{6.5in}(1in,3.5in)
\centering
\LARGE\bfseries \noindent \submiterInstitute\\
\vskip 0.1in
\noindent Basilisk Technical Memorandum\\
\vskip 0.1in
\noindent \MakeUppercase{\ModuleName}
\end{textblock*}

~\\
\vskip 4.25in

}

%
%	setup float environment spacing
%
\setlength\textfloatsep{5pt}
\setlength{\abovecaptionskip}{3pt}
\renewcommand{\figurename}{Fig.}

%
%	cancel the date formatting
%
\date{}


%
%	Rename the Bibliography section label
%
\renewcommand{\refname}{REFERENCES}

%
%	Miscellaneous definitions
%

