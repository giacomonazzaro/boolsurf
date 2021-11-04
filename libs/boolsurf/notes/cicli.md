# Intro
Noi vogliamo trovare winding vectors delle celle per poter calcolare velocemente booleane fra qualsiasi coppia di curve.
Secondo la nostra definizione geometrica, non possiamo calcolare winding numbers rispetto a curve non contraibili, perche' in effetti non sono curve che separano la superficie in due parti.
Se l'utente non selezionasse mai una curva non contraibile come input di una booleana, allora non avremmo problemi perche' per calcolare una booleana fra la curve `C0` e `C1` non serve tutto il winding vector, ma solo i winding numbers rispetto a `C0` e `C1`.

# Approccio base
Inizialmente possiamo semplicemente dire che permettiamo operazioni booleane solo fra curve contraibili.
Per fare ció, dobbiamo saper distinguere quali sono tali curve, ma questo lo sappiamo fare.
Le curve non contraibili sono quelle che nel grafo delle celle sono associate ad archi che creano cicli (ogni arco nel grafo e' associato a una curva). 
Quindi e' come se le curve non contraibili non ci fossereo. Hanno solo l'effetto di creare piu' celle under the hood, ma le celle sono un concetto invisibile all'utente, che interagisce solo con curve.
Geometricamente tutto mantiene lo stesso senso, la definizione di destra/sinistra rimane la stessa.

# Approccio di fallback
Possiamo offrire un comportamento di fallback per quando l'utente sceglie una curva non contraibile `C` come input di una booleana. A quel punto gli viene chiesto di indicare uno o piu' punti di superficie che per lui sono interni a `C`. Le celle contenenti quei punti vengono considereti interne a `C` ai fini della booleana.

# Propagazione euristica
In aggiunta, prima di fare la booleana, si puo' dedurre quali altre celle sono ragiovolmente interne alla stessa curva `C`, basandosi sulla selezione dell'utente e seguendo un'euristica.
Data la selezione dell'utente, che forza gli winding number rispetto a C in alcune celle, tale informazione viene propagata alle altre celle come al solito, ma usando la semantica che tutti le curve non contraibili sono tagli della superficie. In questo modo viene solo propagato il winding number della curva non contraibile, gli winding vector precedentemente calcolati rimangono.
Questo a livello algoritmico rimuove tutti i cicli dal grafo, il che assicura che la propagazione del winding number termina sempre senza ambiguita' o casi strani.
Nei casi che abbiamo testato, il risultato a livello di editing e' sempre quello che si aspetta l'utente.