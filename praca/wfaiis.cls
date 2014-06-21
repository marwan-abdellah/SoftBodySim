%% Autor: Marcin Woli\'nski, wolinski@gust.org.pl, mwbook.cls
%% Zmodyfikowana przez: Rados\l aw Klimas, gervazy89@gmail.com
\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{wfaiis}
              [2012/01/10 v0.7
 A LaTeX document class (RK)]
\typeout{To jest oficjalna klasa LaTeX Wydzialu Fizyki, Astronomii i Informatyki Stosowanej na Uniwersytecie Mikolaja Kopernika w Toruniu}
\RequirePackage{color} %%wymaga pakietu color
\RequirePackage{ifthen} %%wymaga pakietu ifthen
\newcommand\@ptsize{}
\newif\if@restonecol
\newcommand\polskiee{\fontencoding{OT4}\selectfont \symbol{166}} %%polecenie wstawiaj¹ce polskie ê
\newcommand{\prace}[1] %%polecenie wstawiaj¹ce tekst w zale¿noœci od polecenia \rodzajpracy
{\ifthenelse{\equal{#1}{in\.zynierska}}{in\.zynierskiej}{}%
\ifthenelse{\equal{#1}{licencjacka}}{licencjackiej}{}%
\ifthenelse{\equal{#1}{magisterska}}{magisterskiej}{}}

\newif\if@openright
\newif\if@mainmatter \@mainmattertrue
\newif\if@appendix \@appendixfalse
\newif\ifHeadingNumbered
\ifx\l@nohyphenation\@undefined
        \newlanguage\l@nohyphenation
\fi
\DeclareOption{a4paper} %%opcje
   {\setlength\paperheight {297mm}%
    \setlength\paperwidth  {210mm}}
\DeclareOption{12pt}{\renewcommand\@ptsize{2}\def\@basesize{12}}
\DeclareOption{oneside}{\@twosidefalse \@mparswitchfalse}
\DeclareOption{twoside}{\@twosidetrue  \@mparswitchtrue}
\DeclareOption{draft}{\setlength\overfullrule{5pt}}
\DeclareOption{final}{\setlength\overfullrule{0pt}}
\@openrightfalse
\DeclareOption{openright}{\@openrighttrue}
\DeclareOption{openany}{\@openrightfalse}
\DeclareOption{openbib}{%
  \AtEndOfPackage{%
   \renewcommand\@openbib@code{%
      \advance\leftmargin\bibindent
      \itemindent -\bibindent
      \listparindent \itemindent
      \parsep \z@
      }%
   \renewcommand\newblock{\par}}%
}
\newcommand*\sectsettings{\bfseries}
\newcommand*\titlesettings{}
\DeclareOption{rmheadings}{%
  \def\sectsettings{\rmfamily\bfseries}%
  \def\titlesettings{\rmfamily}%
  }
\DeclareOption{sfheadings}{%
  \def\sectsettings{\sffamily\bfseries}%
  \def\titlesettings{\sffamily}%
  }
\newcommand*\captionsettings{\small}
\DeclareOption{floatssmall}{%
  \def\figuresettings{\small}%
  \def\tablesettings{\small}%
  }
\DeclareOption{floatsnormalsize}{%
  \def\figuresettings{\normalsize}%
  \def\tablesettings{\normalsize}%
  }
\newif\if@authorfirst
\DeclareOption{indentfirst}{\@afterindenttrue}
\DeclareOption{noindentfirst}{\@afterindentfalse}
\ExecuteOptions{a4paper,12pt,twoside,onecolumn,%
        final,openright,indentfirst,floatsnormalsize} %%opcje domyœlne
\ProcessOptions
\renewcommand\normalsize{%
   \@setfontsize\normalsize\@xiipt{14.5}%
   \abovedisplayskip 12\p@ \@plus3\p@ \@minus7\p@
   \abovedisplayshortskip \z@ \@plus3\p@
   \belowdisplayshortskip 6.5\p@ \@plus3.5\p@ \@minus3\p@
   \belowdisplayskip \abovedisplayskip
   }
\normalsize
\newcommand\small{%
   \@setfontsize\small\@xipt{13.6}%
   \abovedisplayskip 11\p@ \@plus3\p@ \@minus6\p@
   \abovedisplayshortskip \z@ \@plus3\p@
   \belowdisplayshortskip 6.5\p@ \@plus3.5\p@ \@minus3\p@
   \belowdisplayskip \abovedisplayskip
}
\newcommand\footnotesize{%
   \@setfontsize\footnotesize\@xpt\@xiipt
   \abovedisplayskip 10\p@ \@plus2\p@ \@minus5\p@
   \abovedisplayshortskip \z@ \@plus3\p@
   \belowdisplayshortskip 6\p@ \@plus3\p@ \@minus3\p@
   \belowdisplayskip \abovedisplayskip
}
\newcommand\scriptsize{\@setfontsize\scriptsize\@viiipt{9.5}}
\newcommand\tiny{\@setfontsize\tiny\@vipt\@viipt} %%definicje wielkoœci czcionek
\newcommand\large{\@setfontsize\large\@xivpt{18}}
\newcommand\Large{\@setfontsize\Large\@xviipt{22}}
\newcommand\LARGE{\@setfontsize\LARGE\@xxpt{24}}
\newcommand\huge{\@setfontsize\huge\@xxvpt{30}}
\let\Huge=\huge
\def\secondarysize{\small}
\setlength\smallskipamount{.25\baselineskip \@plus.125\baselineskip
        \@minus.0625\baselineskip}
\setlength\medskipamount{.5\baselineskip \@plus.25\baselineskip
        \@minus.125\baselineskip}
\setlength\bigskipamount{1\baselineskip \@plus.5\baselineskip
        \@minus.25\baselineskip}
\setlength\headheight{1cm}
\setlength\headsep{0.5cm}
\setlength\hoffset{-1in}
\setlength\topskip{\@basesize\p@}
\setlength\footskip{1cm}
\setlength\maxdepth{.5\topskip}
\setlength\marginparsep{0cc}
\setlength\marginparwidth{0cc}
\setlength\oddsidemargin{3cm}
\def\marginpar{\ClassError{wfaiis}{Notatki na marginesie sa zablokowane w tej klasie.\MessageBreak}{}\global\let\marginpar\@gobble}
\setlength\columnsep{2cc}
\setlength\columnseprule{0\p@}
     \setlength\textwidth{15.5cm}
  \@settopoint\textwidth
  \setlength\textheight{25cm}
\setlength\@tempdima{\paperwidth}
\addtolength\@tempdima{-\textwidth}
\addtolength\@tempdima{-\oddsidemargin}
\addtolength\@tempdima{-\marginparwidth}
\addtolength\@tempdima{-\marginparsep}
%\divide\@tempdima by2

\setlength\evensidemargin{\@tempdima}
%\if@twoside
%  \setlength\evensidemargin{\oddsidemargin}
%  \setlength\oddsidemargin{@tempdima}
%\fi
 \setlength\topmargin{1cm}
 \setlength\voffset{-1in}
\setlength\footnotesep{8.4\p@}
\setlength{\skip\footins}{10.8\p@ \@plus 4\p@ \@minus 2\p@}
\setlength\floatsep    {12\p@ \@plus 2\p@ \@minus 4\p@}
\setlength\textfloatsep{20\p@ \@plus 2\p@ \@minus 4\p@}
\setlength\intextsep   {14\p@ \@plus 4\p@ \@minus 4\p@}
\setlength\dblfloatsep    {14\p@ \@plus 2\p@ \@minus 4\p@}
\setlength\dbltextfloatsep{20\p@ \@plus 2\p@ \@minus 4\p@}
\setlength\@fptop{0\p@ \@plus 1fil}
\setlength\@fpsep{10\p@ \@plus 2fil}
\setlength\@fpbot{0\p@ \@plus 1fil}
\setlength\@dblfptop{0\p@ \@plus 1fil}
\setlength\@dblfpsep{10\p@ \@plus 2fil}
\setlength\@dblfpbot{0\p@ \@plus 1fil}
\def\markboth#1#2{%
  \begingroup
    \let\label\relax \let\index\relax \let\glossary\relax
    \unrestored@protected@xdef\@themark {{#1}{#2}}%
    \@temptokena \expandafter{\@themark}%
    \mark{\the\@temptokena}%
  \endgroup
  \if@nobreak\ifvmode\nobreak\fi\fi}
\def\markright#1{%
  \begingroup
    \let\label\relax \let\index\relax \let\glossary\relax
    \expandafter\@markright\@themark {#1}%
    \@temptokena \expandafter{\@themark}%
    \mark{\the\@temptokena}%
  \endgroup
  \if@nobreak\ifvmode\nobreak\fi\fi}
\def\@markright#1#2#3{\@temptokena {#1}%
  \unrestored@protected@xdef\@themark{{\the\@temptokena}{#3}}}
\def\secondarysize{\small}
\setlength\lineskip{1\p@}
\setlength\normallineskip{1\p@}
\renewcommand\baselinestretch{}
\setlength\parskip{\z@}
\if@twocolumn
  \setlength\parindent{1em}
\else
  \setlength\parindent{1.5em}
\fi
\newdimen\@parindent
\@parindent\parindent
\@lowpenalty   51
\@medpenalty  151
\@highpenalty 301
\hyphenpenalty=100
\exhyphenpenalty=10000
\widowpenalty 10000
\clubpenalty  5000
 \displaywidowpenalty 5000
\def\@fnsymbol#1{\ensuremath{\ifcase#1\or *\or **\or *{*}*
  \else\@ctrerr\fi}}
\setcounter{topnumber}{5}
\renewcommand\topfraction{.8}
\setcounter{bottomnumber}{5}
\renewcommand\bottomfraction{.8}
\setcounter{totalnumber}{5}
\renewcommand\textfraction{.2}
\renewcommand\floatpagefraction{.5}
\setcounter{dbltopnumber}{2}
\renewcommand\dbltopfraction{.7}
\renewcommand\dblfloatpagefraction{.5}
%%
\newcommand\hf@empty{%
  \let\@oddhead\@empty \let\@oddfoot\@empty
  \let\@evenhead\@empty \let\@evenfoot\@empty}

\newcommand\hf@plain{%
  \let\@oddhead\@empty
  \def\@oddfoot{\reset@font\hfil\thepage\hfil}%
  \let\@evenhead\@empty
  \let\@evenfoot\@oddfoot}
\newcommand\hf@outer{%
  \let\@oddhead\@empty
  \def\@oddfoot{\reset@font\hfil\thepage}%
  \let\@evenhead\@empty
  \def\@evenfoot{\reset@font\thepage\hfil}}
\newcommand\heading@font{\small\itshape}
\newcommand\hf@headings{%
  \def\@oddhead{\reset@font{\heading@font\rightmark}\hfil\thepage}%
  \let\@oddfoot\@empty
  \def\@evenhead{\reset@font\thepage\hfil\heading@font\leftmark}%
  \let\@evenfoot\@empty
  }
\newcommand\hf@uheadings{%
  \def\@oddhead{\vtop{%
      \hb@xt@\textwidth{%
        \reset@font{\heading@font\rightmark}\hfil\thepage\strut}%
      \vskip2pt\hrule\vskip-2.4pt}}%
  \let\@oddfoot\@empty
  \def\@evenhead{\vtop{%
      \hb@xt@\textwidth{%
        \reset@font\strut\thepage\hfil\heading@font\leftmark}%
      \vskip2pt\hrule\vskip-2.4pt}}%
  \let\@evenfoot\@empty
  }
\def\ps@empty{%
  \let\@mkboth\@gobbletwo
  \let\ps@normal\hf@empty
  \let\ps@opening\hf@empty
  \let\ps@closing\hf@empty
  \let\ps@blank\hf@empty
  \ps@normal}
\def\ps@plain{%
  \let\@mkboth\@gobbletwo
  \let\ps@normal\hf@plain
  \let\ps@opening\hf@plain
  \let\ps@closing\hf@plain
  \let\ps@blank\hf@empty
  \ps@normal}
\def\ps@outer{%
  \let\@mkboth\@gobbletwo
  \let\ps@normal\hf@outer
  \let\ps@opening\hf@empty
  \let\ps@closing\hf@empty
  \let\ps@blank\hf@empty
  \ps@normal}
\if@twoside
  \def\ps@headings{%
    \let\@mkboth\markboth
    \let\ps@normal\hf@headings
    \let\ps@opening\hf@empty
    \let\ps@closing\hf@headings
    \let\ps@blank\hf@empty
    \ps@normal
    \def\chaptermark##1{%
      \markboth{%
        \ifHeadingNumbered
     \if@mainmatter
     \@chapapp\
            \thechapter.\enspace
          \fi
        \fi
        ##1}{}}%
    \def\sectionmark##1{%
      \markright{%
        \ifHeadingNumbered \thesection.\enspace \fi
        ##1}}}
\else
  \def\ps@headings{%
    \let\@mkboth\markboth
    \let\ps@normal\hf@headings
    \let\ps@opening\hf@empty
    \let\ps@closing\hf@headings
    \let\ps@blank\hf@empty
    \ps@normal
    \def\chaptermark##1{%
      \markright{%
        \ifHeadingNumbered
   \if@mainmatter
     \@chapapp\
            \thechapter.\enspace
   \fi
        \fi
        ##1}}}
\fi
\def\ps@myheadings{%
    \let\ps@normal\hf@headings
    \let\ps@opening\hf@empty
    \let\ps@closing\hf@headings
    \let\ps@blank\hf@empty
    \ps@normal
    \let\@mkboth\@gobbletwo
    \let\chaptermark\@gobble
    \let\sectionmark\@gobble
    }
\if@twoside
  \def\ps@uheadings{%
    \let\@mkboth\markboth
    \let\ps@normal\hf@uheadings
    \let\ps@opening\hf@empty
    \let\ps@closing\hf@uheadings
    \let\ps@blank\hf@empty
    \ps@normal
    \def\chaptermark##1{%
      \markboth{%
        \ifHeadingNumbered
   \if@mainmatter
     \@chapapp\
            \thechapter.\enspace
   \fi
        \fi
        ##1}{}}%
    \def\sectionmark##1{%
      \markright{%
        \ifHeadingNumbered \thesection.\enspace \fi
        ##1}}}
\else
  \def\ps@uheadings{%
    \let\@mkboth\markboth
    \let\ps@normal\hf@uheadings
    \let\ps@opening\hf@empty
    \let\ps@closing\hf@uheadings
    \let\ps@blank\hf@empty
    \ps@normal
    \def\chaptermark##1{%
      \markright{%
        \ifHeadingNumbered
   \if@mainmatter
     \@chapapp\
            \thechapter.\enspace
   \fi
        \fi
        ##1}}}
\fi
\def\ps@myuheadings{%
    \let\ps@normal\hf@uheadings
    \let\ps@opening\hf@empty
    \let\ps@closing\hf@uheadings
    \let\ps@blank\hf@empty
    \ps@normal
    \let\@mkboth\@gobbletwo
    \let\chaptermark\@gobble
    \let\sectionmark\@gobble
    }
\let\@date\@empty
\def\album#1{\\nr albumu: #1\def\@album{\relax}} %%definicje i sprawdzanie wymaganych poleceñ
\def\zaklad#1{\def\@zaklad{#1}}
\def\rodzajpracy#1{\def\@rodzajpracy{#1}}
\def\autor#1{\def\@autor{#1}}
\def\tytul#1{\def\@tytul{#1}}
\def\kierunek#1{\def\@kierunek{#1}}
\def\promotor#1{\def\@promotor{#1}}
\def\jednostkapromotora#1{\def\@jednostkapromotora{#1}}
\def\rok#1{\def\@rok{#1}}
\def\podziekowania#1{\def\@podziekowania{#1}}
\def\@album{\\{\color{red} Podaj nr albumu}
\ClassError{wfaiis}{Brakuje polecenia \noexpand\album}{}}
\def\@autor{{\color{red} Podaj swoje imie i nazwisko}
\ClassError{wfaiis}{Brakuje polecenia \noexpand\autor}{}}
\def\@tytul{{\color{red} Podaj temat swojej pracy dyplomowej}
\ClassError{wfaiis}{Brakuje polecenia \noexpand\tytul}{}}
\def\@zaklad{{\color{red} Podaj zak\l ad promotora}
\ClassError{wfaiis}{Brakuje polecenia \noexpand\zaklad}{}}
\def\@rodzajpracy{{\color{red} Podaj rodzaj pracy}
\ClassError{wfaiis}{Brakuje polecenia \noexpand\rodzajpracy}{}}
\def\@kierunek{{\color{red} Podaj nazwe kierunku}
\ClassError{wfaiis}{Brakuje polecenia \noexpand\kierunek}{}}
\def\@promotor{{\color{red} Podaj imie i nazwisko promotora}
\ClassError{wfaiis}{Brakuje polecenia \noexpand\promotor}{}}
\def\@jednostkapromotora{{\color{red} Podaj jednostke promotora}
\ClassError{wfaiis}{Brakuje polecenia \noexpand\jednostkapromotora}{}}
\def\@rok{{\color{red} Podaj rok obrony}
\ClassError{wfaiis}{Brakuje polecenia \noexpand\rok}{}}

\newcommand\maketitle{\par
  \begingroup
    \renewcommand\thefootnote{\@fnsymbol\c@footnote}%
    \long\def\@makefntext##1{
        \parindent\@parindent
        \@textsuperscript{\normalfont\@thefnmark}\enspace##1}%
      \newpage
      \global\@topnum\z@   % Prevents figures from going at top of page.
      \@maketitle
    \thispagestyle{opening}\@thanks
  \endgroup
\setcounter{footnote}{0}%
  \global\let\thanks\relax
  \global\let\maketitle\relax
  \global\let\@maketitle\relax
  \global\let\@thanks\@empty
  \global\let\@author\@empty
  \global\let\@date\@empty
  \global\let\@title\@empty
  \global\let\title\relax
  \global\let\author\relax
  \global\let\date\relax
  \global\let\and\relax
}
\def\@maketitle{% %%definicja stron pocz¹tkowych
  \newpage
  \begin{center}
    {\Large Uniwersytet Miko\l aja Kopernika \\ Wydzia\l\  Fizyki, Astronomii i Informatyki Stosowanej \\ \@zaklad \par}
	\vskip 2.1cm
	{{\begin{tabular}[t]{c}\Large \@autor\@album\end{tabular}}\par}
	\vskip 1cm
	{Praca \@rodzajpracy \\ na kierunku \@kierunek \par}
	\vskip 2cm
	{\huge \@tytul \par}%
	\vskip 1.5cm%
	\begin{flushright}
	\begin{minipage}{8cm}
	{Opiekun pracy dyplomowej \\ \@promotor \\ \@jednostkapromotora \par}%
	\end{minipage}
	\end{flushright}
	\vfill%
	{\large{Toru\'n \@rok} \par}%
	\vskip 1cm%
	\begin{tabular}{@{}cc@{}}
	\begin{minipage}{7.5cm}
	\begin{center}
	Prac\polskiee\ przyjmuj\polskiee\ i akceptuj\polskiee  \\ 
	\ \\
	\parbox{5cm}{\dotfill}\\
	\small\emph{data i podpis opiekuna pracy}\\
	\end{center}
	\end{minipage}
	&
    \begin{minipage}{7.5cm}
	\begin{center}
	Potwierdzam z\l o\.zenie pracy dyplomowej \\ 
	\ \\
	\parbox{5cm}{\dotfill}\\
	\small\emph{data i podpis pracownika dziekanatu}\\
	\end{center}
	\end{minipage}
	\end{tabular}    
  \end{center}%
  \thispagestyle{empty}
  \ifx\@podziekowania\@undefined
  \else
          \cleardoublepage
  \vspace*{12cm}\vfill
  \null\hfill\begin{minipage}{7cm}
  {\fontfamily{cmr}\fontsize{10pt}{10} \selectfont \@podziekowania}
 \end{minipage}\par\vfill
\fi


  \newpage
\null
\vfill  
  \begin{minipage}{14cm}
   \begin{center}
     {\small\it Uniwersytet Miko\l aja Kopernika  zastrzega sobie prawo w\l asno\'sci niniejszej pracy
            \prace{\@rodzajpracy} w~celu udost\polskiee pniania dla potrzeb dzia\l alno\'sci naukowo-badawczej lub dydaktycznej}
 \end{center}
 \end{minipage}
  \newpage
  \par
  \vskip 1em
  }
\def\and{%                  % \begin{tabular}
  \end{tabular}%
  \quad
  \begin{tabular}[t]{c}%
  \large\strut}%   % \end{tabular}
\def\@sect#1#2#3#4#5#6[#7]#8{%
  \ifnum #2>\c@secnumdepth
    \let\@svsec\@empty
  \else
    \refstepcounter{#1}%
    \protected@edef\@svsec{\@seccntformat{#1}\relax}%
  \fi
  \@tempskipa #5\relax
  \ifdim \@tempskipa>\z@
    \begingroup
      \normalfont\sectsettings#6{%
        \language\l@nohyphenation \raggedright
        \interlinepenalty \@M
        \leftskip #3\noindent
        \@svsec\ignorespaces
        #8\@@par}%
    \endgroup
    \csname #1mark\endcsname{#7}%
    \addcontentsline{toc}{#1}{%
      \ifnum #2>\c@secnumdepth \else
        \protect\numberline{\csname the#1\endcsname.}%
      \fi
      #8}%
  \else
    \def\@svsechd{%
      \normalfont\sectsettings#6{\hskip #3\relax
      \@svsec\ignorespaces #8}%
      \csname #1mark\endcsname{#7}%
      \addcontentsline{toc}{#1}{%
        \ifnum #2>\c@secnumdepth \else
          \protect\numberline{\csname the#1\endcsname.}%
        \fi
        #8}}%
  \fi
  \@xsect{#5}}
\def\@ssect#1#2#3#4#5{%
  \@tempskipa #3\relax
  \ifdim \@tempskipa>\z@
    \begingroup
      \normalfont\sectsettings#4{%
        \language\l@nohyphenation \raggedright
        \interlinepenalty \@M
        \leftskip #1\noindent\ignorespaces
        #5\@@par}%
    \endgroup
  \else
    \def\@svsechd{\normalfont\sectsettings#4{\hskip #1\relax #5}}%
  \fi
  \@xsect{#3}}
\newcommand*\partmark[1]{\markboth{}{}}
\newcommand*\chaptermark[1]{}
\setcounter{secnumdepth}{2}
\newcounter {part}
\newcounter {chapter}
\newcounter {section}[chapter]
\newcounter {subsection}[section]
\newcounter {subsubsection}[subsection]
\newcounter {paragraph}[subsubsection]
\newcounter {subparagraph}[paragraph]
\renewcommand \thepart {\@Roman\c@part} %%zmiana formatowania liczników
\renewcommand \thechapter {\@arabic\c@chapter}
\renewcommand \thesection {\thechapter.\@arabic\c@section}
\renewcommand\thesubsection   {\thesection.\@arabic\c@subsection}
\renewcommand\thesubsubsection{\thesubsection .\@arabic\c@subsubsection}
\renewcommand\theparagraph    {\thesubsubsection.\@arabic\c@paragraph}
\renewcommand\thesubparagraph {\theparagraph.\@arabic\c@subparagraph}
\def\part@level{-1}
\def\chapter@level{0}
\def\section@level{1}
\def\subsection@level{2}
\def\subsubsection@level{3}
\def\paragraph@level{4}
\def\subparagraph@level{5}
\newcommand\@chapapp{\chaptername}
\renewcommand*\@seccntformat[1]{\csname the#1\endcsname.\enspace}
\def\cleardoublepage{\clearpage
  \if@twoside
    \ifodd\c@page\else
      \hbox{}\thispagestyle{blank}\newpage
      \if@twocolumn\hbox{}\newpage\fi
    \fi
  \fi}
\newcommand\frontmatter{%
  \cleardoublepage
  \@mainmatterfalse
  \pagenumbering{Roman}}
\newcommand\mainmatter{%
  \cleardoublepage
  \@mainmattertrue
  \pagenumbering{arabic}}
\newcommand\backmatter{%
  \if@openright
    \cleardoublepage
  \else
    \clearpage
  \fi
  \@mainmatterfalse}
\newcommand*\mw@getflags[1]{%
  \expandafter\expandafter\expandafter
  \mw@getflagsx\csname#1@flags\endcsname}
\newcommand*\mw@getflagsx[4]{%
  \def\mw@HeadingRunIn{1#1}%
  \def\mw@HeadingBreakBefore{1#2}%
  \def\mw@HeadingBreakAfter{1#3}%
  \def\mw@HeadingWholeWidth{1#4}%
}
\newcommand*\mw@setflags[1]{%
  \expandafter\edef\csname#1@flags\endcsname{%
    \if\mw@HeadingRunIn 1\else 0\fi
    \if\mw@HeadingBreakBefore 1\else 0\fi
    \if\mw@HeadingBreakAfter 1\else 0\fi
    \if\mw@HeadingWholeWidth 1\else 0\fi
  }
 }
\newcommand*\mw@section[1]{%
  \mw@getflags{#1}%
  \if@noskipsec \leavevmode \fi
  \par
  \if@nobreak \everypar{}\fi
  \@ifstar
    {\HeadingNumberedfalse\@dblarg{\mw@sectionx{#1}}}%
    {\HeadingNumberedtrue\@dblarg{\mw@sectionx{#1}}}}
\def\mw@sectionx#1[#2]{%
  \@dblarg{\mw@sectionxx{#1}{#2}}}
\def\mw@sectionxx#1#2[#3]#4{%
  \if\mw@HeadingBreakBefore
    \if@specialpage\else\thispagestyle{closing}\fi
 \if@openright\cleardoublepage\else
            \clearpage
 \fi
    \if\mw@HeadingBreakAfter
      \thispagestyle{blank}\else
      \thispagestyle{opening}\fi
       \global\@topnum\z@
    \fi
  \edef\mw@HeadingLevel{\csname #1@level\endcsname
        \space}% space delimits level number!
  \ifHeadingNumbered
      \ifnum \mw@HeadingLevel>\c@secnumdepth \HeadingNumberedfalse \fi
 \if@mainmatter\else \HeadingNumberedfalse \fi
  \fi
  \ifHeadingNumbered
    \refstepcounter{#1}%
    \protected@edef\HeadingNumber{\expandafter\noexpand
\csname the#1\endcsname\relax}%
  \else
    \mw@hyperref@phantom{#1}%
    \let\HeadingNumber\@empty
  \fi
  \def\HeadingRHeadText{#2}%
  \def\HeadingTOCText{#3}%
  \def\HeadingText{#4}%
  \def\mw@HeadingType{#1}%
  \if\mw@HeadingRunIn
    \mw@runinheading
  \else
    \if\mw@HeadingWholeWidth
      \if@twocolumn
        \if\mw@HeadingBreakAfter
        \onecolumn
        \mw@normalheading
        \pagebreak\relax
              \if@twoside
                \null
                \thispagestyle{blank}%
                \newpage
              \fi
        \twocolumn
        \else
          \@topnewpage[\mw@normalheading]%
          \mw@markandtoc
        \fi
      \else
        \mw@normalheading
        \if\mw@HeadingBreakAfter\pagebreak\relax\fi
      \fi
    \else
      \mw@normalheading
      \if\mw@HeadingBreakAfter\pagebreak\relax\fi
    \fi
  \fi
  }
\newcommand*\mw@markandtoc{%
     \csname \mw@HeadingType mark\endcsname{\HeadingRHeadText}%
     \ifnum \mw@HeadingLevel>\c@tocdepth \else
       \expandafter\ifx\csname \mw@HeadingType @toc\endcsname\relax
         \addcontentsline{toc}{\mw@HeadingType }{\ifHeadingNumbered
           \protect\numberline{\mw@seccntformat{\HeadingNumber}}\fi
           \HeadingTOCText}%
       \else
         \csname \mw@HeadingType @toc\endcsname
       \fi
     \fi
  }%
\def\mw@seccntformat#1{#1.\enspace}
\newcommand*\mw@normalheading{%
  \if\mw@HeadingBreakBefore \hrule\@height\z@\nobreak
     \else \addpenalty\@secpenalty \fi
  \addvspace{\csname \mw@HeadingType @preskip\endcsname}%
  \bgroup\language\l@nohyphenation
   \raggedright
   \interlinepenalty\@M
   \normalfont\sectsettings
   \csname \mw@HeadingType @head\endcsname
   \@@par
  \egroup
  \ifnum0<%
    \if@twocolumn0\else1\fi
    \if\mw@HeadingWholeWidth0\else1\fi
    \if\mw@HeadingBreakAfter1 \else0 \fi
  \mw@markandtoc\fi
  \nobreak
  \vskip\csname\mw@HeadingType @postskip\endcsname
  \@afterheading
 }
\newcommand*\mw@runinheading{%
    \addvspace{\csname \mw@HeadingType @preskip\endcsname}%
    \@nobreakfalse
    \global\@noskipsectrue
    \everypar{%
      \if@noskipsec
        \global\@noskipsecfalse
       {\setbox\z@\lastbox}%
        \clubpenalty\@M
        \bgroup\language\l@nohyphenation
           \normalfont\sectsettings
           \csname \mw@HeadingType @head\endcsname
        \egroup
        \mw@markandtoc
        \hspace{\csname\mw@HeadingType @postskip\endcsname}%
      \else
        \clubpenalty \@clubpenalty
        \everypar{}%
      \fi}
}
\let\@titleindent\z@
\newcommand*\FormatBlockHeading[1]{%
  \leftskip\@titleindent
  #1{\noindent
  \ifHeadingNumbered\mw@seccntformat\HeadingNumber\fi
  \ignorespaces\HeadingText\@@par}
  }
\newcommand*\FormatHangHeading[1]{%
  #1{%
    \@hangfrom{\hskip\@titleindent
        \ifHeadingNumbered\mw@seccntformat\HeadingNumber\fi}%
    \ignorespaces\HeadingText\@@par}%
  }
\newcommand*\FormatChapterHeading[3]{% %%formatowanie nag³ówka rozdzia³u
  \ifHeadingNumbered
   #2\HeadingNumber.\par\nobreak
   \vspace{#1}%
  \fi
  #3\HeadingText\par\nobreak
  }
\newcommand*\FormatRigidChapterHeading[4]{%
  \nointerlineskip
  \vbox to#1{\color@begingroup
    \vfil
    \ifHeadingNumbered
     #3\HeadingNumber.\enspace%%\par\nobreak
    \fi
    #4\HeadingText%%\par\nobreak
    \color@endgroup
  }%
}
\newcommand*\FormatRunInHeading[1]{%
  \hskip\@titleindent
  #1{%
  \ifHeadingNumbered\mw@seccntformat\HeadingNumber\fi
  \ignorespaces\HeadingText\unskip}%
}
\newcommand*\SetSectionFormatting[5][default]{%
 \def\mw@HeadingRunIn{10}\def\mw@HeadingBreakBefore{10}%
 \def\mw@HeadingBreakAfter{10}\def\mw@HeadingWholeWidth{10}%
 \mw@processflags#1,\relax
 \expandafter\def\csname #2\endcsname{\mw@section{#2}}%
 \expandafter\def\csname #2@preskip\endcsname{#3}%
 \expandafter\def\csname #2@head\endcsname{#4}%
 \expandafter\def\csname #2@postskip\endcsname{#5}%
 \mw@setflags{#2}%
 }
\newcommand*\DeclareSectioningCommand[3][default]{%
        \SetSectionFormatting[#1]{#2}}
\def\mw@processflags#1,#2\relax{%
  \expandafter\ifx\csname sec@@flag@#1\endcsname\relax
        \ClassError{wfaiis}{Unknown sectioning flag `#1'}{}\else
  \csname sec@@flag@#1\endcsname\fi
  \if,#2,\else \mw@processflags#2\relax \fi
  }
\def\sec@@flag@default{}
\def\sec@@flag@runin{\def\mw@HeadingRunIn{11}}
\def\sec@@flag@breakbefore{\def\mw@HeadingBreakBefore{11}}
\def\sec@@flag@breakafter{\def\mw@HeadingBreakAfter{11}}
\def\sec@@flag@wholewidth{\def\mw@HeadingWholeWidth{11}}
\SetSectionFormatting[breakbefore,breakafter,wholewidth]{part}
        {0\p@ plus1fil}
        {\FormatChapterHeading{20\p@}%
                {\centering\large\partname\space}%
                {\centering\LARGE}}
        {0\p@ plus1fil}
\SetSectionFormatting[breakbefore,wholewidth]{chapter}
        {0\p@}
        {\FormatRigidChapterHeading{1.4\baselineskip}{12\p@}%
{\LARGE\if@appendix\@chapapp\ \fi\space}{\LARGE}} %%formatowanie dla dodatków
        {1.6\baselineskip}
\newcommand*\chapter@toc{%
\ifHeadingNumbered\typeout{\@chapapp\space\thechapter.}\fi 
  \addcontentsline{toc}{chapter}{%
        \ifHeadingNumbered
\mw@seccntformat{\if@appendix\@chapapp\ \fi\HeadingNumber}% %%dalej dodatki
        \fi
        \HeadingTOCText}%
  \addtocontents{lof}{\protect\addvspace{0\p@}}% %%pliki zawieraj¹ce treœci
  \addtocontents{lot}{\protect\addvspace{0\p@}}%
  \addtocontents{lod}{\protect\addvspace{0\p@}}%
  \addtocontents{loc}{\protect\addvspace{0\p@}}%
  }
\SetSectionFormatting{section}
        {24\p@\@plus5\p@\@minus2\p@}
        {\FormatHangHeading{\large}}
        {10\p@\@plus3\p@}
\SetSectionFormatting{subsection}
        {16\p@\@plus4\p@\@minus1\p@}
        {\FormatHangHeading{}}
        {6\p@\@plus2\p@}
\SetSectionFormatting{subsubsection}
        {10\p@\@plus2\p@\@minus1\p@}
        {\FormatHangHeading{}}
        {4\p@\@plus1\p@}
\SetSectionFormatting[runin]{paragraph}
        {\bigskipamount}
        {\FormatRunInHeading{}}
        {.5em}
\SetSectionFormatting[runin]{subparagraph}
        {\medskipamount}
        {\FormatRunInHeading{}}
        {.5em}
\setlength\leftmargini{1\parindent}
\leftmargin  \leftmargini
\setlength\leftmarginii  {\parindent}
\setlength\leftmarginiii {\parindent}
\setlength\leftmarginiv  {\parindent}
\setlength\leftmarginv  {1em}
  \setlength\leftmarginvi {1em}
\setlength  \labelsep  {.5em}
\setlength  \labelwidth{\leftmargini}
\addtolength\labelwidth{-\labelsep}
\setlength\partopsep{0\p@}
\@beginparpenalty -\@lowpenalty
\@endparpenalty   -\@lowpenalty
\@itempenalty     -\@lowpenalty
\def\@listi{\leftmargin\leftmargini
  \parsep  \z@
  \topsep  .5\baselineskip plus .25\baselineskip minus
           .125\baselineskip
  \itemsep \z@}
\@listi
\def\@listii {\leftmargin\leftmarginii
              \labelwidth\leftmarginii
              \advance\labelwidth-\labelsep
              \topsep  \z@
              \parsep  \z@
              \itemsep \z@}
\def\@listiii{\leftmargin\leftmarginiii
              \labelwidth\leftmarginiii
              \advance\labelwidth-\labelsep
              \itemsep   \z@}
\def\@listiv {\leftmargin\leftmarginiv
              \labelwidth\leftmarginiv
              \advance\labelwidth-\labelsep}
\def\@listv  {\leftmargin\leftmarginv
              \labelwidth\leftmarginv
              \advance\labelwidth-\labelsep}
\def\@listvi {\leftmargin\leftmarginvi
              \labelwidth\leftmarginvi
              \advance\labelwidth-\labelsep}
\def\enumerate{%
  \ifnum \@enumdepth >\thr@@\@toodeep\else
    \advance\@enumdepth\@ne
    \edef\@enumctr{enum\romannumeral\the\@enumdepth}%
      \expandafter
      \list
        \csname label\@enumctr\endcsname
        {\partopsep\topsep \topsep\z@ %MW
        \usecounter\@enumctr\def\makelabel##1{##1\hfil}}%
  \fi}
\newenvironment*{enumerate*}{%
  \ifnum \@enumdepth >\thr@@\@toodeep\else
    \advance\@enumdepth\@ne
    \edef\@enumctr{enum\romannumeral\the\@enumdepth}%
      \expandafter
      \list
        \csname label\@enumctr\endcsname
        {\partopsep\topsep \topsep\z@ %MW
        \leftmargin\z@ \labelwidth\z@
        \itemindent\@parindent \advance\itemindent\labelsep
        \usecounter\@enumctr\def\makelabel##1{##1\hfil}}%
  \fi}
  {\endlist}
\renewcommand\theenumi{\@arabic\c@enumi}
\renewcommand\theenumii{\@alph\c@enumii}
\renewcommand\theenumiii{\@roman\c@enumiii}
\renewcommand\theenumiv{\@Alph\c@enumiv}
\newcommand\labelenumi{\theenumi.}
\newcommand\labelenumii{\theenumii)}
\newcommand\labelenumiii{\theenumiii.}
\newcommand\labelenumiv{\theenumiv.}
\renewcommand\p@enumii{\theenumi-}
\renewcommand\p@enumiii{\theenumi-\theenumii-}
\renewcommand\p@enumiv{\p@enumiii\theenumiii-}
\def\itemize{%
  \ifnum \@itemdepth >\thr@@\@toodeep\else
    \advance\@itemdepth\@ne
    \edef\@itemitem{labelitem\romannumeral\the\@itemdepth}%
    \expandafter
    \list
      \csname\@itemitem\endcsname
      {\partopsep\topsep \topsep\z@ %MW
      \def\makelabel##1{##1\hfil}}%
  \fi}
\newenvironment*{itemize*}{%
  \ifnum \@itemdepth >\thr@@\@toodeep\else
    \advance\@itemdepth\@ne
    \edef\@itemitem{labelitem\romannumeral\the\@itemdepth}%
    \expandafter
    \list
      \csname\@itemitem\endcsname
      {\partopsep\topsep \topsep\z@ %MW
      \leftmargin\z@ \labelwidth\z@
      \itemindent\@parindent \advance\itemindent\labelsep
      \def\makelabel##1{##1\hfil}}%
  \fi}
  {\endlist}
\newcommand\labelitemi{\textemdash}
\newcommand\labelitemii{\textemdash}
\newcommand\labelitemiii{\textemdash}
\newcommand\labelitemiv{\textemdash}
\newenvironment{description}
        {\list{}{\partopsep\topsep \topsep\z@ %MW
         \labelwidth\z@ \itemindent-\leftmargin
         \let\makelabel\descriptionlabel}}
        {\endlist}
\newcommand*\descriptionlabel[1]{\hspace\labelsep
                                \normalfont\bfseries #1}
\newenvironment{verse}
               {\let\\\@centercr
                \list{}{\itemsep      \z@
                        \parsep \medskipamount
                        \itemindent   -1.5em%
                        \listparindent\itemindent
                        \rightmargin  \leftmargin
                        \advance\leftmargin 1.5em}%
                \item\relax}
               {\endlist}
\newenvironment{quotation}
               {\list{}{\listparindent 1.5em%
                        \itemindent    \listparindent
                        \rightmargin   \leftmargin
                        \parsep        \z@ \@plus\p@}%
                \item\relax}
               {\endlist}
\newenvironment{quote}
               {\list{}{\rightmargin\leftmargin}%
                \item\relax}
               {\endlist}
\def\@begintheorem#1#2{\trivlist
   \item[\hskip \labelsep{\bfseries #1\ #2.}]\itshape}
\def\@opargbegintheorem#1#2#3{\trivlist
      \item[\hskip \labelsep{\bfseries #1\ #2.\ (#3)}]\itshape}
\newenvironment{titlepage}
    {%
      \cleardoublepage
      \@restonecolfalse\newpage
      \thispagestyle{blank}%
      \setcounter{page}\@ne
    }%
    {\if@restonecol\twocolumn \else \newpage \fi
     \if@twoside\else
        \setcounter{page}\@ne
     \fi
    }
\newcommand\appendix{\@appendixtrue\par
  \setcounter{chapter}{0}%
  \setcounter{section}{0}%
  \gdef\@chapapp{\appendixname}%
  \gdef\thechapter{\@Alph\c@chapter}}
\setlength\arraycolsep{5\p@}
\setlength\tabcolsep{6\p@}
\setlength\arrayrulewidth{.4\p@}
\setlength\doublerulesep{2\p@}
\setlength\tabbingsep{\labelsep}
\skip\@mpfootins = \skip\footins
\setlength\fboxsep{3\p@}
\setlength\fboxrule{.4\p@}
\renewcommand\theequation %%licznik równañ
 {\@arabic\c@equation}
\def\@floatboxreset{%
  \reset@font
  \normalsize
  \@setminipage
  \csname\@captype settings\endcsname
}
\newcounter{figure} %%modyfikacje i tworzenie poszczególnych œrodowisk
\renewcommand \thefigure
     {\@arabic\c@figure}
\def\fps@figure{tbp}
\def\ftype@figure{1}
\def\ext@figure{lof}
\def\fnum@figure{\figurename~\thefigure}
\newenvironment{figure}
               {\@float{figure}}
               {\end@float}
\newenvironment{figure*}
               {\@dblfloat{figure}}
               {\end@dblfloat}
\newcounter{table}
\renewcommand \thetable
     {\@arabic\c@table}
\def\fps@table{tbp}
\def\ftype@table{2}
\def\ext@table{lot}
\def\fnum@table{\tablename~\thetable}
\newenvironment{table}
               {\@float{table}}
               {\end@float}
\newenvironment{table*}
               {\@dblfloat{table}}
               {\end@dblfloat}
			   
\newcounter{diagram}
\renewcommand \thediagram
     {\@arabic\c@diagram}
\def\fps@diagram{tbp}
\def\ftype@diagram{1}
\def\ext@diagram{lod}
\def\fnum@diagram{\diagramname~\thediagram}
\newenvironment{diagram}
               {\@float{diagram}}
               {\end@float}
\newenvironment{diagram*}
               {\@dblfloat{diagram}}
               {\end@dblfloat}			

\newcounter{chart}
\renewcommand \thechart
     {\@arabic\c@chart}
\def\fps@chart{tbp}
\def\ftype@chart{1}
\def\ext@chart{loc}
\def\fnum@chart{\chartname~\thechart}
\newenvironment{chart}
               {\@float{chart}}
               {\end@float}
\newenvironment{chart*}
               {\@dblfloat{chart}}
               {\end@dblfloat}		   
			   
\def\centeredlast{%
  \let\\\@centercr
  \rightskip\z@\@plus-.5fil
  \leftskip\z@\@plus.5fil
  \parindent\z@
  \parfillskip\@flushglue
  }
\newlength\abovecaptionskip
\newlength\belowcaptionskip
\setlength\abovecaptionskip{.5\baselineskip}
\setlength\belowcaptionskip{.5\baselineskip}
\long\def\@caption#1[#2]#3{%
  \par
  \addcontentsline{\csname ext@#1\endcsname}{#1}%
    {\protect\numberline{\csname the#1\endcsname.}{\ignorespaces #2}}%
  \begingroup
    \@parboxrestore
    \if@minipage
      \@setminipage
    \fi
    \captionsettings
    \@makecaption{\csname fnum@#1\endcsname}{\ignorespaces #3}\par
  \endgroup}
\long\def\@makecaption#1#2{%
  \addvspace\abovecaptionskip
  \bgroup\centeredlast
    #1.\enspace#2\par
  \egroup
  \vskip\belowcaptionskip}
\DeclareOldFontCommand{\rm}{\normalfont\rmfamily}{\mathrm}
\DeclareOldFontCommand{\sf}{\normalfont\sffamily}{\mathsf}
\DeclareOldFontCommand{\tt}{\normalfont\ttfamily}{\mathtt}
\DeclareOldFontCommand{\bf}{\normalfont\bfseries}{\mathbf}
\DeclareOldFontCommand{\it}{\normalfont\itshape}{\mathit}
\DeclareOldFontCommand{\sl}{\normalfont\slshape}{\@nomath\sl}
\DeclareOldFontCommand{\sc}{\normalfont\scshape}{\@nomath\sc}
\DeclareRobustCommand*\cal{\@fontswitch\relax\mathcal}
\DeclareRobustCommand*\mit{\@fontswitch\relax\mathnormal}
\newcommand\@pnumwidth{1.55em}
\newcommand\@tocrmarg{2.55em plus 2cm}
\newcommand\@dotsep{4.5}
\setcounter{tocdepth}{2}
\newcommand\tableofcontents{%
      \@restonecolfalse
  {\@tempcnta=\c@tocdepth
   \c@tocdepth=-1
\chapter*%
        {\contentsname}%
        \protect\@mkboth{\contentsname}{\contentsname}%
     \c@tocdepth=\@tempcnta
    \secondarysize
    \gdef\previous@toc@level{-1000}%
    \@starttoc{toc}}%
    }
\def\mw@tocskip#1#2{%
     \ifnum#1>\previous@toc@level
        \nobreak
     \else\ifnum#1<\previous@toc@level
        \addpenalty\@secpenalty
     \fi\fi
    \gdef\previous@toc@level{#1}%
    \vskip #2\relax
}
\def\mw@tocline#1#2#3#4#5{%
  \ifnum #1>\c@tocdepth \else
    \mw@tocskip{#1}{\z@ \@plus.2\p@}%
    {\leftskip #3\relax
     \@tempdima #3\relax
     \advance\@tempdima -#2\relax
     \rightskip \@tocrmarg \parfillskip -\rightskip
     \interlinepenalty\@M
     \noindent\kern-\@tempdima{\ignorespaces#4}\nobreak
     \leaders\hbox{$\m@th
        \mkern \@dotsep mu\hbox{.}\mkern \@dotsep
        mu$}\hfill
     \nobreak
     \hb@xt@\@pnumwidth{\hfil\normalfont \normalcolor #5}%
     \par}%
  \fi}
\newcommand*\SetTOCIndents[1]{\bgroup
  \reset@font\secondarysize
  \setbox\@tempboxa\hbox{}%
  \set@tocindentsX
{}%
#1\relax
  \egroup
  }
\def\set@tocindentsX#1#2{%
  \ifx\relax#2\expandafter\@gobble\else
  \setbox\@tempboxa\hbox{\unhbox\@tempboxa#2}%
  \expandafter\xdef\csname @tocindent#1\endcsname{\the\wd\@tempboxa}%
  \expandafter\set@tocindentsX
  \fi
  {#1i}%
  }
\SetTOCIndents{{}{\quad}{9.10.\enspace}{9.9.10.\enspace}{9.9.9.10.\enspace}{\quad}{\quad}}
\def\numberline#1{\hb@xt@\@tempdima{#1\hfil}\ignorespaces}
\newcommand*\l@part[2]{%
  \ifnum \c@tocdepth >-2\relax
    \mw@tocskip{-1}{.6\baselineskip}%
    \begingroup
      \parindent \z@ \rightskip \@pnumwidth
      \parfillskip -\@pnumwidth
      {\leavevmode\normalsize
        \def\numberline##1{##1\enspace}%
        \hfil\bfseries #1\hfil\null %\hb@xt@\@pnumwidth{\hss #2}%
        }\par
       \nobreak
         \global\@nobreaktrue
         \everypar{\global\@nobreakfalse\everypar{}}%
    \endgroup
  \addvspace{.4\baselineskip}%
  \fi}
\newcommand*\l@chapter[2]{\addvspace{.3\baselineskip}%
        \mw@tocline{0}{\@tocindent}{\@tocindenti}{\textbf{#1}}{#2}%
        \addvspace{.2\baselineskip}}
\newcommand*\l@section{\mw@tocline{1}{\@tocindenti}{\@tocindentii}}
\newcommand*\l@subsection{\mw@tocline{2}{\@tocindentii}{\@tocindentiii}}
\newcommand*\l@subsubsection{\mw@tocline{3}{\@tocindentiii}{\@tocindentiiii}}
\newcommand*\l@paragraph{\mw@tocline{4}{\@tocindentiiii}{\@tocindentiiiii}}
\newcommand*\l@subparagraph{\mw@tocline{5}{\@tocindentiiiii}{\@tocindentiiiiii}}
\newcommand\listoffigures{% %%spisy obiektów ze œrodowisk
      \@restonecolfalse
    \chapter*{\listfigurename}
    {\secondarysize
    \gdef\previous@toc@level{-1000}%
    \@starttoc{lof}}%
    }
\newcommand*\l@figure{\mw@tocline{1}{0pt}{2.5em}}
\newcommand\listoftables{%
      \@restonecolfalse
    \chapter*{\listtablename}
    {\secondarysize
    \gdef\previous@toc@level{-1000}%
    \@starttoc{lot}}%
    }
\let\l@table\l@figure

\newcommand\listofcharts{%
      \@restonecolfalse
    \chapter*{\listchartname}
    {\secondarysize
    \gdef\previous@toc@level{-1000}%
    \@starttoc{loc}}%
    }
\let\l@chart\l@figure

\newcommand\listofdiagrams{%
      \@restonecolfalse
    \chapter*{\listdiagramname}
    {\secondarysize
    \gdef\previous@toc@level{-1000}%
    \@starttoc{lod}}%
    }
\let\l@diagram\l@figure

\newcommand\listofcodes{%
      \@restonecolfalse
    \chapter*{\listcodename}
    {\secondarysize
    \gdef\previous@toc@level{-1000}%
    \@starttoc{lol}}%
    }
\let\l@code\l@figure

\newdimen\bibindent
\setlength\bibindent{1.5em}
\newenvironment{thebibliography}[1]{%
        \def
 \chaptermark
        ##1{\@mkboth{##1}{##1}}%
 \chapter*{\bibname}%
      \list{\@biblabel{\@arabic\c@enumiv}}%
           {\settowidth\labelwidth{\@biblabel{#1}}%
            \leftmargin\labelwidth
            \advance\leftmargin\labelsep
            \@openbib@code
            \usecounter{enumiv}%
            \let\p@enumiv\@empty
            \renewcommand\theenumiv{\@arabic\c@enumiv}%
            \secondarysize}%
      \sloppy
      \sfcode`\.\@m}
     {\def\@noitemerr
       {\@latex@warning{Empty `thebibliography' environment}}%
      \endlist}
\newcommand\newblock{\hskip .11em\@plus.33em\@minus.07em}
\let\@openbib@code\@empty
\newenvironment{theindex}
               {
                  \@restonecoltrue
                \columnseprule \z@
                \columnsep 35\p@
                \twocolumn
               \chapter*{\indexname}%
                \@mkboth{\indexname}{\indexname}%
                \thispagestyle{opening}\parindent\z@
                \parskip\z@ \@plus .3\p@\relax
                \let\item\@idxitem
        \secondarysize}
               {\onecolumn}
\newcommand\@idxitem{\par\hangindent 40\p@}
\newcommand\subitem{\@idxitem \hspace*{20\p@}}
\newcommand\subsubitem{\@idxitem \hspace*{30\p@}}
\newcommand\indexspace{\par \vskip 10\p@ \@plus5\p@ \@minus3\p@\relax}
\renewcommand\footnoterule{%
  \kern-3\p@
  \hrule\@width4pc
  \kern2.6\p@}
\@addtoreset{footnote}{chapter}
\newcommand\@makefntext[1]{%
    \parindent\@parindent
    \indent\@makefnmark\enspace#1}
\newcommand\contentsname{Contents} %%nazewnictwo angielskie
\newcommand\listfigurename{List of Figures}
\newcommand\listtablename{List of Tables}
\newcommand\bibname{Bibliography}
\newcommand\indexname{Index}
\newcommand\figurename{Figure}
\newcommand\tablename{Table}
\newcommand\diagramname{Diagram}
\newcommand\codename{Code}
\newcommand\chartname{Chart}
\newcommand\listdiagramname{List of Diagrams}
\newcommand\listcodename{List of Codes}
\newcommand\listchartname{List of Charts}

\newcommand\partname{Part}
\newcommand\chaptername{Chapter}
\newcommand\appendixname{Appendix}

\def\today{\ifcase\month\or
  January\or February\or March\or April\or May\or June\or
  July\or August\or September\or October\or November\or December\fi
  \space\number\day, \number\year}
\pagestyle{plain}
\pagenumbering{arabic}
\flushbottom
\if@twocolumn
  \twocolumn
  \sloppy
\else
  \onecolumn
\fi
\AtBeginDocument{\@ifpackageloaded{natbib}{%
\renewcommand\bibsection{\chapter*{\bibname}%
  \@mkboth{\bibname}{\bibname}}%
}{}}
\let\mw@hyperref@phantom\@gobble
\AtBeginDocument{\@ifpackageloaded{hyperref}{%
    \def\mw@hyperref@phantom#1{%
      \Hy@GlobalStepCount\Hy@linkcounter
      \xdef\@currentHref{#1*.\the\Hy@linkcounter}%
      \Hy@raisedlink{\hyper@anchorstart{\@currentHref}\hyper@anchorend}%
    }%
}{}}
\AtBeginDocument{\@ifpackageloaded{polski}{% %%nazewnictwo polskie
\renewcommand\codename{Listing}
\renewcommand\chartname{Wykres}
\renewcommand\listdiagramname{Spis diagram\'ow}
\renewcommand\listcodename{Spis listing\'ow}
\renewcommand\listchartname{Spis wykres\'ow}
}{}}

\AtBeginDocument{\@ifpackageloaded{listings}{% %%modyfikacja listings
\lstset{captionpos=b,numberbychapter=false}
	\AtBeginDocument{\@ifpackageloaded{polski}{%
	\renewcommand\lstlistlistingname{Spis listing\'ow}
	\renewcommand\lstlistingname{Listing}
	}{\renewcommand\lstlistlistingname{List od Codes}
\renewcommand\lstlistingname{Code}}}

\def\l@lstlisting#1#2{\@dottedtocline{1}{0em}{2.3em}{#1}{#2}}

\def\lst@MakeCaption#1{%
  \lst@ifdisplaystyle
    \ifx #1b%
        \ifx\lst@@caption\@empty\expandafter\lst@HRefStepCounter \else
                                \expandafter\refstepcounter
        \fi {lstlisting}%
        \ifx\lst@label\@empty\else \label{\lst@label}\fi
        \let\lst@arg\lst@intname \lst@ReplaceIn\lst@arg\lst@filenamerpl
        \global\let\lst@name\lst@arg \global\let\lstname\lst@name
        \lst@ifnolol\else
            \ifx\lst@@caption\@empty
                \ifx\lst@caption\@empty
                    \ifx\lst@intname\@empty \else \def\lst@temp{ }%
                    \ifx\lst@intname\lst@temp \else
                        \addcontentsline{lol}{lstlisting}\lst@name
                    \fi\fi
                \fi
            \else
                \addcontentsline{lol}{lstlisting}%
                    {\protect\numberline{\thelstlisting.}\lst@@caption}%
            \fi
         \fi
     \fi
    \ifx\lst@caption\@empty\else
        \lst@IfSubstring #1\lst@captionpos
            {\begingroup \let\@@vskip\vskip
             \def\vskip{\afterassignment\lst@vskip \@tempskipa}%
             \def\lst@vskip{\nobreak\@@vskip\@tempskipa\nobreak}%
             \par\@parboxrestore\normalsize\normalfont % \noindent (AS)
             \ifx #1b\allowbreak \fi
             \ifx\lst@title\@empty
                 \lst@makecaption\fnum@lstlisting{\ignorespaces \lst@caption}
             \else
                 \lst@maketitle\lst@title % (AS)
             \fi
             \ifx #1b\allowbreak \fi
             \endgroup}{}%
    \fi
  \fi}

}{}}

\def\@doendpe{\@endpetrue
  \clubpenalty\@clubpenalty
  \def\par{\@restorepar\everypar{}\par\@endpefalse}%
  \everypar{{\setbox\z@\lastbox}\everypar{}\@endpefalse}%
}
\endinput
