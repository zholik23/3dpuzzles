struct ConvCtx {
    IritPrsrObjectStruct     *in;
    IritPrsrObjectStruct     *out;
    IPFreeformConvStateStruct state;
};

// Tessellates freeform geometry and triangulates every polygon over the whole
// forest. IritPrsrConvertFreeFormHierachy CONSUMES the object it is handed and
// returns a (possibly longer) chain, so the caller frees only `out`.
static void doConvert(void *v)
{
    ConvCtx *c = static_cast<ConvCtx *>(v);
    IritPrsrObjectStruct *out = NULL, *tail = NULL, *o = c -> in;

    while (o != NULL) {
        IritPrsrObjectStruct *next = o -> Pnext, *conv;

        o -> Pnext = NULL;
        // TriangleOnly is deliberately FALSE: it routes through
        // IritGeomConvertPolysToTriangles, which access-violates on some
        // valid inputs. Harvester::triangulate does the job instead.
        conv = IritPrsrConvertFreeFormHierachy(o, &c -> state,
                                               FALSE,     /* TriangleOnly */
                                               FALSE);    /* Regularize   */
        while (conv != NULL) {
            IritPrsrObjectStruct *cNext = conv -> Pnext;

            conv -> Pnext = NULL;
            if (tail != NULL) {
                tail -> Pnext = conv;
                tail = conv;
            }
            else {
                out = tail = conv;
            }
            conv = cNext;
        }
        o = next;
    }
    c -> out = out;
}