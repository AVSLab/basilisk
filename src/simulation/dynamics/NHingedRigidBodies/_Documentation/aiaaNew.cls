%%
%% This is file `aiaa.cls',
%% generated with the docstrip utility.
%%
%% The original source files were:
%%
%% aiaa.dtx  (with options: `class')
%% 
%% The "aiaa" distribution
%% by Bil Kleb <w.l.kleb@larc.nasa.gov>.
%% See the file `aiaa.ins' for information
%% on how you may (re-)distribute the `aiaa' bundle of files.
%% 
%% Description: a bundle of LaTeX and BibTeX files to produce
%%              AIAA papers and simulated journal articles/notes
%% Keywords: LaTeX, class, AIAA, BibTeX, bibliographic-style, 9pt-option
%% Author: Bil Kleb <w.l.kleb@larc.nasa.gov>
%% Maintainer: same
%% Latest Version: 1.0 <mar 1997>
%%
%% Please see the information in file `aiaa.ins' on how you
%% may use and (re-)distribute this file.  Run LaTeX on the file
%% `aiaa.ins' to get the main aiaa class and other auxilary packages.
%% Then run LaTeX on `aiaa.dtx' (this file) to obtain a users manual
%% and code documentation.
%%
%% NOTE: This file may NOT be distributed if not accompanied by 'aiaa.ins'
%% \CharacterTable
%%  {Upper-case    \A\B\C\D\E\F\G\H\I\J\K\L\M\N\O\P\Q\R\S\T\U\V\W\X\Y\Z
%%   Lower-case    \a\b\c\d\e\f\g\h\i\j\k\l\m\n\o\p\q\r\s\t\u\v\w\x\y\z
%%   Digits        \0\1\2\3\4\5\6\7\8\9
%%   Exclamation   \!     Double quote  \"     Hash (number) \#
%%   Dollar        \$     Percent       \%     Ampersand     \&
%%   Acute accent  \'     Left paren    \(     Right paren   \)
%%   Asterisk      \*     Plus          \+     Comma         \,
%%   Minus         \-     Point         \.     Solidus       \/
%%   Colon         \:     Semicolon     \;     Less than     \<
%%   Equals        \=     Greater than  \>     Question mark \?
%%   Commercial at \@     Left bracket  \[     Backslash     \\
%%   Right bracket \]     Circumflex    \^     Underscore    \_
%%   Grave accent  \`     Left brace    \{     Vertical bar  \|
%%   Right brace   \}     Tilde         \~}
\NeedsTeXFormat{LaTeX2e}[1994/06/01]
\ProvidesClass{aiaaNew}[1996/03/03 v1.0 AIAA document class]
\newif\if@submit
\newif\if@paper
\newif\if@article
\newif\if@note
\newif\if@cover
\@submitfalse
\@papertrue
\@articlefalse
\@notefalse
\@coverfalse
\DeclareOption{note}{\@notetrue\@articlefalse\@submitfalse\@paperfalse}
\DeclareOption{article}{\@articletrue\@notefalse\@paperfalse\@submitfalse}
\DeclareOption{paper}{\@papertrue\@articlefalse\@notefalse\@submitfalse}
\DeclareOption{submit}{\@submittrue\@paperfalse\@articlefalse\@notefalse}
\DeclareOption{cover}{\@covertrue}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}%
                \typeout{NOTE: Passing ``\CurrentOption" option on to the
                         standard LaTeX article class}}
\ProcessOptions
\typeout{}
\if@submit%
  \typeout{NOTE: aiaa journal submission mode
          - all other aiaa options ignored}
\else% paper, article, or note
  \if@paper%
    \typeout{TYPESETTING in AIAA conference PAPER format ...}
  \else% article or note
    \if@article%
      \typeout{TYPESETTING in AIAA journal ARTICLE simulation format ...}
    \else% note
      \typeout{TYPESETTING in AIAA journal NOTE simulation ...}
    \fi
  \fi
\fi
\typeout{}
\if@submit%
  \LoadClass[12pt]{article}%
  %\RequirePackage[noheads,tablesfirst,nomarkers]{endfloat}%
  \RequirePackage{setspace}%
  \RequirePackage{lastpage}%
\else% paper, article or note
  \if@paper%
  \LoadClass[twoside, onecolumn]{article}%
  \else
  \LoadClass[twoside,twocolumn]{article}%
  \fi
 % \RequirePackage{dropping}%
  \RequirePackage{lastpage}%
  \if@paper%
    \relax%
  \else% article or note
    \RequirePackage{aiaa9pt}%
  \fi%
\fi
\RequirePackage{graphicx}
%\if@submit%
%	\relax
%\else
	\RequirePackage[colorlinks=true, pdfstartview=FitV, linkcolor= black, citecolor= black, urlcolor=black]{hyperref}
%\fi
\RequirePackage{overcite}
\RequirePackage{caption2}
\RequirePackage{fancyhdr}
\RequirePackage{times}
\RequirePackage{url}
\RequirePackage{amsmath}
\RequirePackage{bm}
\RequirePackage{subfigure}[1995/03/06]
\RequirePackage{tabls}
\if@submit
  \setlength{\topmargin}{-0.25in}
  \setlength{\headsep}{9.5pt}
  \setlength{\textheight}{9in}
  \setlength{\textwidth}{6.5in}
  \setlength{\oddsidemargin}{0.0in}
  \AtBeginDocument{\onehalfspacing}% turn on `doublespacing'
\else% paper, article, or note
  \if@paper
    \setlength{\topmargin}{-.6in}
    \setlength{\headsep}{.15in}
    \setlength{\headheight}{.35in}
    \setlength{\textheight}{9.in}
    \setlength{\textwidth}{6.5in}
    \setlength{\footskip}{0.25in}
    \setlength{\oddsidemargin}{0.0in}
  \else% article or note
    \setlength{\topmargin}{-0.75in}
    \setlength{\headsep}{10pt}
    \setlength{\footskip}{20pt}
    \setlength{\textheight}{10in}
    \setlength{\textwidth}{7in}
    \setlength{\columnsep}{0.375in}
    \setlength{\oddsidemargin}{-0.25in}
    \renewcommand{\baselinestretch}{0.9}
  \fi
  \setlength{\evensidemargin}{\oddsidemargin}
\fi
\newcommand*{\SubmitName}[1]%
            {\def\AA@submitname{#1}}
\newcommand*{\PaperNumber}[1]%
            {\def\AA@papernumber{#1}}
\newcommand*{\ArticleHeader}[1]%
            {\def\AA@articleheader{#1}}
\newcommand*{\ArticleIssue}[1]%
            {\def\AA@articleissue{#1}}
\newcommand*{\JournalName}[1]%
            {\def\AA@journalname{#1}}
\newcommand*{\JournalIssue}[1]%
            {\def\AA@journalissue{#1}}
\newcommand*{\NoteHeader}[1]%
            {\def\AA@noteheader{#1}}
\SubmitName{}
\PaperNumber{}
\ArticleHeader{}
\ArticleIssue{}
\JournalName{}
\NoteHeader{}
\pagestyle{fancy}% note: must be issued after any \textwidth
                 %       command
\renewcommand{\headrulewidth}{0pt}
\renewcommand{\footrulewidth}{0pt}
\fancyhf{} % clear all footers and headers
\if@submit
  \rfoot{\footnotesize\scshape\thepage\ of \pageref{LastPage}}
  \lfoot{\footnotesize\scshape\AA@submitname}
\else% paper, article, or note
  \if@paper
    \cfoot{\footnotesize\scshape\thepage\ of \pageref{LastPage}
           \ifx\AA@papernumber\@empty\relax\else\\
           %\rule[.2\baselineskip]{0.5in}{0.2pt}\\
           American Institute of Aeronautics
           and Astronautics\fi}
           %and Astronautics Paper \AA@papernumber\fi}
  \else% article or note
    \if@article
      \chead{\footnotesize\scshape\MakeUppercase{\AA@articleheader}}
      \fancyhead[RO,LE]{\footnotesize\thepage}
      \fancypagestyle{plain}{%
        \fancyhf{}
        \lhead{\scriptsize{\scshape\AA@journalname}\\\AA@articleissue}
        \cfoot{\footnotesize\thepage}}
    \else% note
      \chead[\scriptsize\scshape\AA@noteheader]%
            {\scriptsize\scshape\AA@noteheader}
      \rhead[]{\footnotesize \thepage}
      \lhead[\footnotesize \thepage]{}
    \fi
  \fi
\fi
\renewcommand{\abstract}[1]%
             {\def\@abstract{#1}}
\abstract{}
\let\@date\@empty
\if@submit%
  %\def\dropword{}
  \def\PARstart#1#2{{#1}\MakeLowercase{#2}}
\else
  %\def\dropword#1#2 {\dropping{2}{\bfseries{} #1}\MakeUppercase{#2} }% spaces important!
  \def\PARstart#1#2{\begingroup\def\par{\endgraf\endgroup\lineskiplimit=0pt}
    \setbox2=\hbox{\uppercase{#2} }\newdimen\tmpht \tmpht \ht2
    \advance\tmpht by \baselineskip\font\hhuge=cmb10 at \tmpht
    \setbox1=\hbox{{\hhuge #1}}
    \count7=\tmpht \count8=\ht1\divide\count8 by 1000 \divide\count7 by\count8
    \tmpht=.001\tmpht\multiply\tmpht by \count7\font\hhuge=cmb10 at \tmpht
    \setbox1=\hbox{{\hhuge #1}} \noindent \hangindent1.05\wd1
    \hangafter=-2 {\hskip-\hangindent \lower1\ht1\hbox{\raise1.0\ht2\copy1}%
    \kern-0\wd1}\copy2\lineskiplimit=-1000pt}
\fi
\doublehyphendemerits=100000
\setlength\hfuzz{2pt}% reduce overfull warnings

\setcounter{errorcontextlines}{10}

	
\renewcommand{\thefootnote}{\alph{footnote}}


\newcommand{\Conference}[1]{\def\AA@conference{#1}}
\Conference{}
\newcommand{\CoverFigure}[1]{\def\AA@coverfigure{#1}}
\CoverFigure{}
\if@submit%
  \newcommand{\makecover}{}
\else% paper, article, or note
  \newcommand{\makecover}{%
  \begin{titlepage}
  \let\AA@sfdefault\sfdefault% save normal fonts
  \let\AA@rmdefault\rmdefault
  \let\AA@ttdefault\ttdefault
  %\renewcommand{\sfdefault}{phv}% change to new fonts
  \renewcommand{\rmdefault}{ptm}
  \renewcommand{\ttdefault}{pcr}
  \enlargethispage{1in}
  \setcounter{page}{0}
  \renewcommand\thanks[1]{}% locally kill the \thanks{} command
  \setlength{\unitlength}{0.1in}% unit of measure for the picture
  \begin{picture}(75,90)(5,5)%  start a 7-1/2'' x 9'' picture:
  \if@paper%
    \linethickness{4pt}
    \put(0,0){\framebox(75,90){}}% make a framed box
    \linethickness{0.5pt}
    \put(-3,88){\incfig[width=3.25in]{aiaalgo}}% aiaa logo at top left
    \put(12,80)% paper number, title, author/location
        {\makebox(0,0)[tl]{\parbox{5in}
            {\sffamily\bfseries
          \Huge AIAA \AA@papernumber% paper number
          \\[5pt]
          \huge\@title%                title
          \\[5pt]
          \Large\mdseries\@author}}}%   author/location
    \ifx\AA@coverfigure\@empty% optional figure
      \relax
    \else
      \put(37.5,14){\makebox(0,0)[b]{%
          \includegraphics[width=5in,height=4in]%
          {\AA@coverfigure}}}%
    \fi
    \put(37.5,2){\makebox(0,0)[b]{\parbox{7.5in}%  meeting, location, and date
        {\centering\huge\bfseries\sffamily%
        \AA@conference}}}
    \put(-0.5,-2){\parbox[t]{7.5in}{\normalsize\sffamily% notice
      For permission to copy or republish, contact the
      American Institute of Aeronautics and Astronautics\\
      1801 Alexander Bell Drive, Suite 500, Reston, VA 22091}}
  \else% article, or note
    \put(3,77.5)% title and authors
        {\makebox(0,0)[tl]{\parbox{6.5in}
         {\sffamily\bfseries
          \huge\@title%
          \\*[-8pt]

          \Large\mdseries\@author}}}
    \put(3,19){\makebox(0,0)[tl]{\parbox{7.5in}%  journal/issue
        {\raggedright\bfseries\sffamily
         Simulated Reprint from\\
        \Huge%
        \AA@journalname\\
        \small\mdseries\AA@journalissue}}}
    \put(3,4.5){\incfig[width=1in]{aiaalgo}}% aiaa logo at bottom left
    \put(3,3)% notice
        {\makebox(0,0)[tl]{\parbox{6.5in}
         {\sffamily
          \slshape A publication of the\\
          \upshape American Institute of Aeronautics and Astronautics, Inc.\\
         1801 Alexander Bell Drive, Suite 500\\
         Reston, VA 22091}}}
  \fi
  \end{picture}
  \renewcommand\sfdefault{\AA@sfdefault}
  \renewcommand\rmdefault{\AA@rmdefault}
  \renewcommand\ttdefault{\AA@ttdefault}
  \end{titlepage}}
\fi
\newcommand{\PaperNotice}[1]{\def\AA@papernotice{#1}}
\PaperNotice{}
\newcommand{\JournalNotice}[1]{\def\AA@journalnotice{#1}}
\JournalNotice{}
\if@submit%
  \newcommand\makenotice{}%
\else% paper, article, or note
  \newcommand\makenotice{%
    \begingroup
    \renewcommand\thefootnote{}
      \if@paper%
        \ifx\AA@papernotice\@empty\else\footnotetext{\AA@papernotice}\fi
      \else% article or note
        \ifx\AA@journalnotice\@empty\else\footnotetext{\AA@journalnotice}\fi
      \fi%
    \endgroup
  }
\fi
\newcommand{\CopyrightA}[1]{Copyright \copyright #1 by the
  American Institute of Aeronautics and Astronautics, Inc. All
  rights reserved.}
\newcommand{\CopyrightB}[2]{Copyright \copyright #1 by
  #2. Published by the American Institute of Aeronautics
  and Astronautics, Inc.\ with permission.}
\newcommand{\CopyrightC}{This paper is a work of the U.S.
  Government and is not subject to copyright protection in the
  United States.}
\newcommand{\CopyrightD}[1]{Copyright \copyright #1 by the
  American Institute of Aeronautics and Astronautics, Inc. No
  copyright is asserted in the United States under Title 17,
  U.S. Code.  The U.S. Government has a royalty-free license to
  exercise all rights under the copyright claimed herein for
  governmental purposes.  All other rights are reserved by the
  copyright owner.}
\if@submit
  \renewcommand\figurename{Figure}
  \renewcommand\captionlabeldelim{.}
\else% paper, article, or note
  \renewcommand\figurename{Figure}% change to the ugly, silly
                                % abbreviatation as per aiaa---saves
                                % all of two characters !?
  \renewcommand\captionlabeldelim{:}
\fi
\if@submit
	\renewcommand\captionfont{\small\bfseries}
\else
	\renewcommand\captionfont{\normalsize\bfseries}
\fi
\renewcommand\thesubfigure{\alph{subfigure})}
\renewcommand\p@subfigure{\thefigure(}
\renewcommand\subcaplabelfont{\captionfont}
\renewcommand{\subfigcapmargin}{2pt}
\if@submit
  \setlength{\abovecaptionskip}{ 6pt plus 2pt minus 2pt}
  \setlength{\belowcaptionskip}{ 10pt plus 1pt minus 1pt}
  \renewcommand{\subfigcapskip}{.2\abovecaptionskip}%    seem to
  \renewcommand{\subfigbottomskip}{.7\belowcaptionskip}% have no effect?
\else
  \setlength{\abovecaptionskip}{ 6pt plus 2pt minus 2pt}
  \setlength{\belowcaptionskip}{ 3pt plus 1pt minus 1pt}
  \renewcommand{\subfigcapskip}{.2\abovecaptionskip}%    seem to
  \renewcommand{\subfigbottomskip}{.7\belowcaptionskip}% have no effect?
\fi
% modify float spacing
  \renewcommand{\topfraction}{0.9}
  \renewcommand{\bottomfraction}{0.9}
  \renewcommand{\textfraction}{0.1}
  \renewcommand{\floatpagefraction}{0.8}% 0.5 is the default
  \renewcommand{\dbltopfraction}{\floatpagefraction}
  \renewcommand{\dblfloatpagefraction}{\floatpagefraction}
  \setcounter{topnumber}{10}
  \setcounter{dbltopnumber}{\value{topnumber}}
  \setcounter{bottomnumber}{\value{topnumber}}
  \setcounter{totalnumber}{\value{topnumber}}
  \addtocounter{totalnumber}{\value{bottomnumber}}
  \setlength{\floatsep}        { 5pt plus 2pt minus 2pt}
  \setlength{\textfloatsep}    { 5pt plus 2pt minus 3pt}
  \setlength{\intextsep}       { 5pt plus 2pt minus 2pt}
  \setlength{\dblfloatsep}     {\floatsep}
  \setlength{\dbltextfloatsep} {\textfloatsep}

\renewcommand{\@biblabel}[1]{$^{#1}$}
\renewenvironment{thebibliography}[1]%
     {\section*{\refname\@mkboth{\MakeUppercase\refname}%
                                 {\MakeUppercase\refname}}%
      \list{\@biblabel{\@arabic\c@enumiv}}%
           {\setlength{\leftmargin}{0pt}%
            \settowidth{\labelwidth}{\@biblabel{#1}}%
            \setlength{\itemindent}{\parindent}%
            \advance\itemindent by \labelwidth%
            \setlength{\labelsep}{0.0em}%
            \setlength{\itemsep}{-\smallskipamount}%
            \@openbib@code%
            \usecounter{enumiv}%
            \let\p@enumiv\@empty%
            \renewcommand\theenumiv{\@arabic\c@enumiv}%
            \if@submit%
              \normalsize%
            \else%
              \footnotesize%
            \fi}
      \sloppy\clubpenalty4000\widowpenalty4000%
      \sfcode`\.\@m}
      {\def\@noitemerr
        {\@latex@warning{Empty `thebibliography' environment}}%
      \endlist}
\newcommand{\incfig}{\centering\includegraphics}
%\setkeys{Gin}{width=\linewidth}
%\setkeys{Gin}{width=\linewidth,\keepaspectratio}
\if@submit%
   %\setkeys{Gin}{totalheight=0.35\textheight}
   \relax
\else% paper, article, or note
   \relax
\fi
\newlength{\sfm@width}%               Subfigure width
\newlength{\sfm@colsep}%              Subfigure column separation
\setlength{\sfm@colsep}{2\tabcolsep}% Use twice tabular column separation
\newcounter{sfm@count}%               Item count
\newenvironment{subfigmatrix}[1]{%
  \begingroup%
  \centering%
  \vspace*{-\subfigtopskip}% remove the vertical spacing inserted
                           % by the subfigure package
    %
    % Save the "real" subfigure macro and start the item counter off
    % at -1 to detect the first item.
    % Set the \sfm@width to the single element size.
    %
    \let\sfm@subfigure\subfigure%
    \setcounter{sfm@count}{-1}%
    \setlength{\sfm@width}{\linewidth}%
    \addtolength{\sfm@width}{\sfm@colsep}%
    \addtolength{\sfm@width}{-#1\sfm@colsep}%
    \divide\sfm@width by#1
    %\setkeys{Gin}{width=\sfm@width}%
    %\setkeys{Gin}{width=\sfm@width,\keepaspectratio}%
    %
    % Redefine the \subfigure and \subtable macros locally to this
    % environment so that we can wrap them with minipages.
    %
    \def\subfigure{% try the transpose
      \ifnum \value{sfm@count} = -1
      % very first item
        \setcounter{sfm@count}{1}%
      \else% Not very first item
        \addtocounter{sfm@count}{1}%
        \ifnum \value{sfm@count} = 1
          % Beginning of next column, finish the last column.
          \\%
        \else%
          % middle or last item
          \hfill%
          \ifnum #1 = \value{sfm@count}%
            % Reset the counter of at the end of the row.
            \setcounter{sfm@count}{0}%
          \fi%
        \fi%
      \fi%
      \sfm@subfigure}%
    \let\subtable\subfigure%
  }{%
    \\%
  \endgroup}%
\if@paper
  \renewcommand{\and}{\\[-.9\baselineskip]}
\else
  \renewcommand{\and}{\\*[-8pt]}
\fi
\renewcommand{\maketitle}{%
  \if@cover
    \makecover
  \else
    \relax
  \fi
  \par
  \begingroup
    \renewcommand\thefootnote{\@fnsymbol\c@footnote}%
    \def\@makefnmark{\rlap{\@textsuperscript{\normalfont\@thefnmark}}}%
    \long\def\@makefntext##1{\parindent 1em\noindent
            \hb@xt@1.8em{%
                \hss\@textsuperscript{\normalfont\@thefnmark}}##1}%
  \global\@topnum\z@   % Prevents figures from going at top of page.
  \if@submit%
    \@maketitle%
  \else% paper, article, or note
    \if@note%
      \@maketitle%
    \else% paper article -- should already be in twocolumn mode
      \ifnum \col@number=\@ne% if in column one, good:
         \@maketitle
      \else% not in column one, so start new page:
        \twocolumn[\@maketitle]
      \fi
      \if@article%
        \thispagestyle{plain}
      \fi
    \fi
  \fi
  \@thanks
  \endgroup
  \suppressfloats
  \setcounter{footnote}{0}%
  \makenotice
  \global\let\maketitle\relax
  \global\let\@maketitle\relax
  \global\let\@abstract\@empty
  \global\let\@thanks\@empty
  \global\let\thanks\relax
  \global\let\@author\@empty
  \global\let\author\relax
  \global\let\@title\@empty
  \global\let\title\relax
  \global\let\@date\@empty
  \global\let\abstract\relax
  \global\let\date\relax
}
\def\@maketitle{%
  \newpage
  \begin{center}%
  \let \footnote \thanks
    {\if@paper%
		%\hfill\sffamily\bfseries\large AIAA-\AA@papernumber \\ 
		~ \vskip 0.3in
	\fi
	\LARGE\bf \@title \par}%
    \vskip 2em%
    {\lineskip .5em%
      \large %
      \begin{tabular}[t]{c}%
        \@author
      \end{tabular}\par}%
    \vskip 2em%
    \if@submit
      \newenvironment{AA@abstract}
        {\list{}{\listparindent 1.5em%
                \itemindent \listparindent
                \leftmargin 0.5in%
                \rightmargin \leftmargin
                \parsep \z@ \@plus \p@}%
                \item\relax}
         {\endlist}
         \ifx\@abstract\@empty
           \relax
         \else
           \begin{AA@abstract}
             \small\textbf\@abstract
           \end{AA@abstract}
         \fi 
    \else% paper, article, or note
      \if@note
        \relax
      \else
        \newenvironment{AA@abstract}
        {\list{}{\listparindent 1.em%
                \itemindent \listparindent
                \leftmargin 0.5in%
                \rightmargin \leftmargin
                \parsep \z@ \@plus \p@}%
                \item\relax}
         {\endlist}
         \ifx\@abstract\@empty
           \relax
         \else
           \begin{AA@abstract}
             \small\bfseries\@abstract
           \end{AA@abstract}
         \fi
      \fi
    \fi
  \end{center}%
  \if@submit%
    \ifx\@abstract\@empty
      \relax
    \else
      %\section{Abstract} 
      %\@abstract%
      \relax
    \fi
  \else% paper, article, or note
    \par%
    \if@note%
      \vskip -\medskipamount%
    \else%
      \vskip 1em%
    \fi%
  \fi%
}
%\renewcommand{\thesection}{\Roman{section}.}
%\renewcommand{\thesubsection}{\Alph{subsection}.}
%\renewcommand{\thesubsubsection}{\arabic{subsubsection}.}
 \renewcommand\thesection      {\@Roman\c@section}
 \renewcommand\thesubsection   {\thesection.\@Alph\c@subsection}
 \renewcommand\thesubsubsection{\thesubsection.\@arabic\c@subsubsection}
 \renewcommand\theparagraph    {\thesubsubsection.\@alph\c@paragraph}
 \renewcommand\thesubparagraph {\theparagraph.\@roman\c@subparagraph}
 \setcounter{secnumdepth}{3}% no counters \paragraph and below
%\setcounter{secnumdepth}{-2}% instead of having to use the *'d
                            % section commands
\renewcommand\section{\@startsection
  {section}%                    % section name
  {1}%                          % level
  {\z@}%                        % indentation of heading
  {1.8ex}%                      % before skip (neg, no parindent)
  {0.5ex}%                      % after skip (neg, run-on heading space)
  {\normalfont\center\large\bfseries}}
\renewcommand\subsection{\@startsection{subsection}{2}%
  {\z@}%
  {1.5ex}%
  {0.5ex}%
  {\normalfont\normalsize\bfseries\raggedright}}
\renewcommand\subsubsection{\@startsection{subsubsection}{3}%
  {\z@}%
  {1.5ex}%
  {0.5ex}%
  {\normalfont\normalsize\itshape\raggedright}}
\renewcommand\paragraph{\@startsection{paragraph}{4}%
  {\parindent}%
  {0.3ex}%
  {-1em}%
  {\normalfont\normalsize\sffamily}}
\renewcommand\subparagraph{\@startsection{subparagraph}{5}%
  {\parindent}%
  {0.3ex}%
  {-1em}%
  {\normalfont\normalsize\itshape}}
\listfiles
\endinput


%%
%% End of file `aiaa.cls'.
