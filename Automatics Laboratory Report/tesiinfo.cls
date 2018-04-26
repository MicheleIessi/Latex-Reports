%%
%% This is file `tesiinfo.cls',
%% generated with the docstrip utility.
%%
%% The original source files were:
%%
%% tesiinfo.dtx  (with options: `class')
%% 
%% IMPORTANT NOTICE:
%% 
%% For the copyright see the source file.
%% 
%% Any modified versions of this file must be renamed
%% with new filenames distinct from tesiinfo.cls.
%% 
%% For distribution of the original source see the terms
%% for copying and modification in the file tesiinfo.dtx.
%% 
%% This generated file may be distributed as long as the
%% original source files, as listed above, are part of the
%% same distribution. (The sources need not necessarily be
%% in the same archive or directory.)
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
%%
\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{tesiinfo}
[2008/02/06 v0.1b
  Standard per le Tesi in Ingegneria Informatica -- Università degli Studi dell'Aquila by Gianlorenzo D'Angelo & Mauro Felice]

\usepackage{graphicx}
\newif\ifesterno
\newcommand{\setesterno}{\newif\ifesterno
\esternotrue}
\newcommand{\unsetesterno}{\newif\ifesterno
\esternofalse}

\newif\ifcorrelatoreuno
\newcommand{\setcorrelatoreuno}{\newif\ifcorrelatoreuno
\correlatoreunotrue
\correlatoreduefalse}
\newcommand{\unsetcorrelatoreuno}{\newif\ifcorrelatoreuno
\correlatoreunofalse}

\newif\ifcorrelatoredue
\newcommand{\setcorrelatoredue}{\newif\ifcorrelatoredue
\correlatoreduetrue
\correlatoreunofalse}
\newcommand{\unsetcorrelatoredue}{\newif\ifcorrelatoredue
\correlatoreduefalse}

\newif\ifindice
\newcommand{\setindice}{\newif\ifindice
\indicetrue}
\newcommand{\unsetindice}{\newif\ifindice
\indicefalse}

\newif\ifbibliography
\newcommand{\setbibliography}{\newif\ifbibliography
\bibliographytrue}
\newcommand{\unsetbibliography}{\newif\ifbibliography
\bibliographyfalse}

\newif\ifappendix
\newcommand{\setappendix}{\newif\ifappendix
\appendixtrue}
\newcommand{\unsetappendix}{\newif\ifappendix
\appendixfalse}


\newif\ifspecialistica
\newcommand{\setspecialistica}{\newif\ifspecialistica
\specialisticatrue}
\newcommand{\unsetspecialistica}{\newif\ifspecialistica
\specialisticafalse}

\newif\iftesiinfoheadings
\newif\ifhalftesiinfoheadings
\newif\ifsmallerheadings
\DeclareOption{latexheadings}{\tesiinfoheadingsfalse \halftesiinfoheadingsfalse}
\DeclareOption{tesiinfoheadings}{\tesiinfoheadingstrue \halftesiinfoheadingsfalse}
\DeclareOption{halftesiinfoheadings}{\halftesiinfoheadingstrue \tesiinfoheadingsfalse}
\DeclareOption{normalsizeheadings}{\smallerheadingsfalse}
\DeclareOption{smallerheadings}{\smallerheadingstrue}
\ExecuteOptions{halftesiinfoheadings}
\ExecuteOptions{smallerheadings}
\DeclareOption{10pt}{%
 \iffinal%
  \OptionNotUsed%
  \ClassWarningNoLine{tesiinfo}{Font size 10pt not allowed; using 12pt}%
 \else%
  \PassOptionsToClass{10pt}{report}%
 \fi%
}
\DeclareOption{11pt}{%
 \iffinal%
  \OptionNotUsed%
  \ClassWarningNoLine{tesiinfo}{Font size 11pt not allowed; using 12pt}%
 \else%
  \PassOptionsToClass{11pt}{report}%
 \fi
}
\newcommand{\prelim@contents}[1]{}
\DeclareOption{tocprelim}{%
  \renewcommand{\prelim@contents}[1]{\addcontentsline{toc}{section}{#1}}
}
\DeclareOption{draft}{%
  \let\ifdraft\iftrue%
  \let\iffinal\iffalse%
  \PassOptionsToClass{final}{report}%
}
\DeclareOption{final}{%
  \let\ifdraft\iffalse%
  \let\iffinal\iftrue%
  \PassOptionsToClass{final}{report}%
}
\DeclareOption{semifinal}{%
  \let\ifdraft\iffalse%
  \let\iffinal\iftrue%
  \PassOptionsToClass{draft}{report}%
}
\DeclareOption*{%
  \PassOptionsToClass{\CurrentOption}{report}%
}
\ExecuteOptions{final}
\ProcessOptions
\iffinal\LoadClass[12pt]{report}[1994/06/01]\else\LoadClass{report}[1994/06/01]\fi
\renewcommand{\footnotesize}{\normalsize}
\renewcommand{\@makefntext}[1]{%
  \parindent 1em%
  \noindent
  \footnotespacing
  \hbox to 1.8em{\hss\@makefnmark}#1
  \par\normalspacing
}
\renewcommand{\defaultscriptratio}{.75}
\renewcommand{\defaultscriptscriptratio}{.6}
\setlength\headsep{25\p@}
\if@twocolumn
\else
  %\setlength\topmargin{-0.4in}
  %\setlength\headheight{0.16667in}
  %\setlength\headsep{0.33333in}
  %\setlength\textheight{9.7in}
  %\setlength\footskip{0.3in}
  %\setlength\oddsidemargin{1cm}
  %\setlength\evensidemargin{1cm}
  %\setlength\textwidth{5.90in}
%\setlength\textwidth{5.30in}
  %\setlength\marginparsep{0.1in}
  %\setlength\marginparwidth{0.8in}
  %\addtolength{\oddsidemargin}{0.87cm}
  %\addtolength{\evensidemargin}{0.87cm}
  \addtolength{\oddsidemargin}{-1mm}
  \addtolength{\evensidemargin}{-1mm}
  \addtolength{\textwidth}{13mm}
  \newlength{\linea}
  \setlength{\linea}{\textwidth}
  \addtolength{\headheight}{12pt}
%  \addtolength{\linea}{-1cm}
\fi
\if@compatibility
  \setlength\topmargin{0\p@}
\else
\fi
\newcommand{\ps@tesiinfo}{%
\def\@oddhead{\noindent\rule[-1.5ex]{\linea}{0.5pt}\\[1.5ex]\hspace{-\linea}
\ifindice \contentsname
\else 
  \ifbibliography \bibname
  \else 
    \ifappendix Appendice \Alph{chapter}
    \else Capitolo \arabic{chapter}
    \fi
  \fi
\fi
}

}

\newcommand{\ps@tesiinfoc}{%
\def\@oddhead{\hfil\thepage\hfil}%
\def\@oddfoot{}%
\def\@evenhead{\@oddhead}%
\def\@evenfoot{}%
\def\chaptermark##1{}%
\def\sectionmark##1{}%
}
\raggedbottom
\renewcommand{\contentsname}{Indice dei Contenuti}
\renewcommand{\l@figure}{\@dottedtocline{1}{1.5em}{2.6em}}
\let\l@table\l@figure

\renewenvironment{thebibliography}[1]
     {\chapter*{\bibname
        \@mkboth{\uppercase{\bibname}}{\uppercase{\bibname}}}%
      \list{\@biblabel{\arabic{enumiv}}}%
           {\settowidth\labelwidth{\@biblabel{#1}}%
            \leftmargin\labelwidth
            \advance\leftmargin\labelsep
            \@openbib@code
            \usecounter{enumiv}%
            \let\p@enumiv\@empty
            \renewcommand\theenumiv{\arabic{enumiv}}}%
      \sloppy\clubpenalty4000\widowpenalty4000%
      \sfcode`\.=\@m\relax
      \addcontentsline{toc}{chapter}{\bibname}
      %\singlespacing}
      }
     {\def\@noitemerr
       {\@latex@warning{Empty `thebibliography' environment}}%
      %\normalspacing
      \endlist}


\newcommand{\@titolo}{}
\newcommand{\@sottotitolo}{}
\newcommand{\@laureando}{}
\newcommand{\@relatore}{}
\newcommand{\@relatoreesterno}{}
\newcommand{\@correlatoreuno}{}
\newcommand{\@correlatoredue}{}
\newcommand{\@annoaccademico}{}

\newcommand{\titolo}[1]{\renewcommand{\@titolo}{#1}}
\newcommand{\sottotitolo}[1]{\renewcommand{\@sottotitolo}{#1}}
\newcommand{\laureando}[1]{\renewcommand{\@laureando}{#1}}
\newcommand{\relatore}[1]{\renewcommand{\@relatore}{#1}}
\newcommand{\relatoreesterno}[1]{\renewcommand{\@relatoreesterno}{#1}}
\newcommand{\correlatoreuno}[1]{\renewcommand{\@correlatoreuno}{#1}}
\newcommand{\correlatoredue}[1]{\renewcommand{\@correlatoredue}{#1}}
\newcommand{\annoaccademico}[1]{\renewcommand{\@annoaccademico}{#1}}
\renewcommand{\contentsname}{Indice dei Contenuti}
\renewcommand{\chaptername}{Capitolo}
\renewcommand{\maketitle}{%
}

\renewenvironment{titlepage}{%
  \typeout{Title page}
  \pagestyle{empty}
  %\doublespacing
}{%
  \normalspacing\clearpage
}


\newenvironment{copyrightpage}{%
  \typeout{Copyleft page}
  \pagestyle{empty}
  \null\vfil
  \begin{center}
}{%
  \end{center}
  \vfil\null
  \clearpage%
}

\newcommand{\makecopyright}{%
  \begin{copyrightpage}
  \copyright\ \@copyrightyear{} \@copyrightholder \\
  ALL RIGHTS RESERVED \\[.5in]
  \end{copyrightpage}%
}
\newcommand{\@copyrightholder}{\@author}
\newcommand{\@copyrightyear}{\@conferralyear}

\newcommand{\copyrightholder}[1]{\renewcommand{\@copyrightholder}{#1}}
\newcommand{\copyrightyear}[1]{\renewcommand{\@copyrightyear}{#1}}

\newcommand{\makepublicdomain}{%
  \begin{copyrightpage}
  This document is in the public domain.
  \end{copyrightpage}%
}


\newlength{\@abhdskip}%
\renewenvironment{abstract}{
  \typeout{Abstract}
  \setlength{\@abhdskip}{.6in}
  \addtolength{\@abhdskip}{-\topmargin}
  \addtolength{\@abhdskip}{-\headheight}
  \addtolength{\@abhdskip}{-\headsep}
  \addtolength{\@abhdskip}{-\baselineskip}
  \begin{center}%
    \vspace*{\@abhdskip}%
    \ifphd%
      {\def\\{\noexpand\\} \xdef\@abstracttitle{\@abstracttitle}}
      \expandafter\uppercase\expandafter{\@abstracttitle} \\[0.5\baselineskip]
      \@author,\ Ph.D. \\
      tesiinfo University\ \@conferralyear%
    \else
      \normalsize \bfseries\selectfont
      ABSTRACT
    \fi%
    \vspace{0.5\baselineskip}%
  \end{center}%
  \pagestyle{empty}
  \thispagestyle{empty}
}{%
  \clearpage
}

\newcommand{\@abstracttitle}{\@title}
\newcommand{\abstracttitle}[1]{\renewcommand{\@abstracttitle}{#1}}


\newenvironment{biosketch}{%
  \typeout{Vita}
  \pagestyle{plain}
  \pagenumbering{roman}
  \setcounter{page}{3}
  \prelim@contents{Biographical Sketch}
  \chapter*{Biographical Sketch}
}{%
  \clearpage
}

\newenvironment{dedication}{%
  \prelim@contents{Dedication}
  \null\vfil
  \begin{center}
}{
  \end{center}
  \vfil\null
  \clearpage
}

\newenvironment{acknowledgements}{%
  \prelim@contents{Acknowledgements}
  \chapter*{Acknowledgements}
}{%
  \cleardoublepage
}

\newcommand{\contentspage}{%
  \setindice
 % \singlespacing
  \label{toc}
  \prelim@contents{\contentsname}
  \tableofcontents
  %\normalspacing
  \clearpage
  \unsetindice

}

\newcommand{\tablelistpage}{%
  \singlespacing
  \label{lot}
  \prelim@contents{\listtablename}
  \listoftables
  \normalspacing
  \clearpage
}

\newcommand{\figurelistpage}{%
  \singlespacing
  \label{lof}
  \prelim@contents{\listfigurename}
  \listoffigures
  \normalspacing
  \clearpage
}
\newcommand{\abbrlist}{%
  \singlespacing
  \prelim@contents{List of Abbreviations}
  \chapter*{List of Abbreviations}
}

\newcommand{\symlist}{%
  \singlespacing
  \prelim@contents{List of Symbols}
  \chapter*{List of Symbols}
}

\newcommand{\preface}{%
  \singlespacing
  \prelim@contents{Preface}
  \chapter*{Preface}
}

\newcommand{\bibliografia}[1]{%
  \newpage
  \setbibliography
  \bibliographystyle{plain}
  \bibliography{#1}
  %\unsetbibliography
}

\newcommand{\appendice}{%
  \newpage
  \unsetbibliography
  \setappendix
  \appendix
  %\unsetappendix
}


\newcommand{\singlespacing}{%
  \let\CS=\@currsize\renewcommand{\baselinestretch}{1.0}\tiny\CS}
\newcommand{\singlespacingplus}{%
  \let\CS=\@currsize\renewcommand{\baselinestretch}{1.25}\tiny\CS}
\newcommand{\doublespacing}{%
  \let\CS=\@currsize\renewcommand{\baselinestretch}{1.75}\tiny\CS}
\newcommand{\realdoublespacing}{%
  \let\CS=\@currsize\renewcommand{\baselinestretch}{2}\tiny\CS}
\newcommand{\normalspacing}{\singlespacingplus}
\newcommand{\footnotespacing}{\singlespacing}
\newcommand{\changespacing}[2]{%
  \renewcommand{#1}{%
    \let\CS=\@currsize\renewcommand{\baselinestretch}{#2}\tiny\CS}%
}
\ifdraft\changespacing{\normalspacing}{1}\fi
\newcommand{\changenormalspacing}[1]{\renewcommand{\normalspacing}{#1}}
\normalspacing

\RequirePackage[Lenny]{fncychap}
\pagestyle{tesiinfo}
\endinput
%%
%% End of file `tesiinfo.cls'.
